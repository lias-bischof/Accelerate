use crate::{CalibrationError, Calibrator, Chain, NATSController, SharedNATS, SharedSQL, SQLController};
use axum::{
    Extension, Json, Router,
    extract::State,
    http::StatusCode,
    response::{
        IntoResponse, Response, Sse,
        sse::{Event, KeepAlive},
    },
    routing::{get, post},
};
use futures_util::{Stream, stream};
use nalgebra::Vector3;
use serde_json::{Value, json};
use std::{convert::Infallible, sync::Arc};
use tokio::net::TcpListener;
use tokio::sync::{Mutex, broadcast};

pub struct RESTController {
    router: Router,
}

impl RESTController {

    pub fn create(controller: NATSController, sql: SQLController) -> Result<Self, async_nats::Error> {
        let chain = Chain(sql.load()?);
        let sharedSQL: SharedSQL = Arc::new(Mutex::new(sql));
        let sharedNATS: SharedNATS = Arc::new(controller);
        let calibrator = Calibrator::create(sharedNATS.clone(), chain);
        let (events, _) = broadcast::channel::<Value>(32);
        let router = Router::new()
            .route("/state", get(state))
            .route("/connection", get(connection))
            .route("/configuration", get(configuration))
            .route("/joints", get(joints))
            .route("/calibration", post(calibration))
            .route("/calibration/receipt", get(receipt))
            .with_state(sharedNATS.clone())
            .layer(Extension(calibrator))
            .layer(Extension(sharedSQL.clone()))
            .layer(Extension(events));
        Ok(RESTController {
            router,
        })
    }

    pub async fn serve(self, address: &str) -> Result<(), async_nats::Error> {
        let listener = TcpListener::bind(address).await?;
        tracing::info!(address = %listener.local_addr()?, "REST API listening");
        axum::serve(listener, self.router).await?;
        Ok(())
    }
}

async fn state(State(controller): State<SharedNATS>) -> Json<Value> {
    let state = controller.state();
    let data = json!({"state": &*state.borrow()});
    Json(data)
}

async fn connection(State(controller): State<SharedNATS>) -> Json<Value> {
    let connection = controller.connection();
    let data = json!({"state": &*connection.borrow()});
    Json(data)
}

async fn configuration(State(controller): State<SharedNATS>) -> Json<Value> {
    let configuration = controller.configuration();
    let rounded = (*configuration.borrow()).map(|angles| {
        angles.map(round3)
    });
    let data = json!({"configuration": rounded});
    Json(data)
}

fn round3(value: f64) -> f64 {
     (value * 1000.0).round() / 1000.0
}

async fn joints(
    State(controller): State<SharedNATS>,
    Extension(sql): Extension<SharedSQL>,
) -> Result<Json<Value>, APIError> {
    let sql = sql.lock().await;
    let configuration = (*controller.configuration().borrow())
        .ok_or(APIError::from(CalibrationError::InvalidConfiguration))?;
    let chain = Chain(sql.load()
        .map_err(CalibrationError::SQLError)
        .map_err(APIError::from)?);
    let positions = chain.positions(&configuration);
    let joints = positions.map(|position| [position.x, position.y, position.z].map(round3));
    let response = json!({"joints": joints});
    Ok(Json(response))
}

async fn calibration(
    Extension(calibrator): Extension<Calibrator>,
    Extension(events): Extension<broadcast::Sender<Value>>,
    Json(target): Json<Vector3<f64>>,
) -> Result<Response, APIError> {
    let task = calibrator.add(target).await.map_err(APIError::from)?;
    let result = task.result;
    let response = json!({"id": task.id});
    tokio::spawn(async move {
        let result = result.await.unwrap_or(Err(CalibrationError::QueueClosed));
        let event = match result {
            Ok(correction) => json!({"task": task.id, "correction": correction, "error": null}),
            Err(error) => json!({"task": task.id, "correction": null, "error": APIError::from(error).1})
        };
        let _ = events.send(event);
    });
    Ok((StatusCode::ACCEPTED, Json(response)).into_response())
}

async fn receipt(
    Extension(events): Extension<broadcast::Sender<Value>>,
) -> Sse<impl Stream<Item = Result<Event, Infallible>>> {
    let stream = stream::unfold(events.subscribe(), |mut receiver| async move {
        let event = match receiver.recv().await {
            Ok(data) => Event::default()
                .event("machine/receipt")
                .data(data.to_string()),
            Err(broadcast::error::RecvError::Lagged(missed)) => {
                Event::default().event("machine/receipt/lagged").data(missed.to_string())
            }
            Err(broadcast::error::RecvError::Closed) => return None
        };
        Some((Ok(event), receiver))
    });
    Sse::new(stream).keep_alive(KeepAlive::default())
}

#[derive(Debug)]
struct APIError(StatusCode, String);

impl From<CalibrationError> for APIError {
    fn from(error: CalibrationError) -> Self {
        match error {
            CalibrationError::InvalidConfiguration => Self(
                StatusCode::SERVICE_UNAVAILABLE,
                "joint configuration invalid".into()
            ),
            CalibrationError::Unreachable => Self(
                StatusCode::UNPROCESSABLE_ENTITY,
                "no configuration found for target".into(),
            ),
            CalibrationError::NATSError(_) => Self(
                StatusCode::BAD_GATEWAY,
                "NATS calibration request failed".into(),
            ),
            CalibrationError::SQLError(_) => Self(
                StatusCode::INTERNAL_SERVER_ERROR,
                "calibration accepted but saving offsets failed".into(),
            ),
            CalibrationError::NoReceipt => Self(
                StatusCode::BAD_GATEWAY,
                "no valid receipt received".into()
            ),
            CalibrationError::QueueFull => Self(
                StatusCode::SERVICE_UNAVAILABLE,
                "calibration queue full".into(),
            ),
            CalibrationError::QueueClosed => Self(
                StatusCode::SERVICE_UNAVAILABLE,
                "calibration service stopped".into(),
            ),
            CalibrationError::Rejected(error) => Self(StatusCode::CONFLICT, error)
        }
    }
}

impl IntoResponse for APIError {
    fn into_response(self) -> Response {
        (self.0, Json(json!({"error": self.1}))).into_response()
    }
}
