use HSSE::rest::RESTController;
use HSSE::{Joint, NATSController, SQLController, logger};
use std::f64::consts::{FRAC_PI_2, PI};

const CHAIN: [[f64; 6]; 6] = [
    [0.0, 0.15, FRAC_PI_2, 0.0, -PI, PI],
    [0.30, 0.0, 0.0, 0.0, -PI, PI],
    [0.25, 0.0, 0.0, 0.0, -PI, PI],
    [0.0, 0.10, FRAC_PI_2, 0.0, -PI, PI],
    [0.0, 0.10, -FRAC_PI_2, 0.0, -PI, PI],
    [0.0, 0.08, 0.0, 0.0, -PI, PI],
];

#[tokio::main]
async fn main() -> Result<(), async_nats::Error> {
    logger::init()?;
    let mut sql = SQLController::create("Accelerate-Configuration.db")?;
    match sql.load() {
        Ok(_) => {}
        Err(rusqlite::Error::QueryReturnedNoRows) => {
            sql.save(&CHAIN.map(|[a, d, α, Δϑ, θmin, θmax]| Joint {
                a, d, α, Δϑ, θmin, θmax,
            }))?;
        }
        Err(error) => return Err(error.into()),
    }
    let nats = NATSController::connect("nats://127.0.0.1:4222").await?;
    let rest = RESTController::create(nats, sql)?;
    rest.serve("127.0.0.1:3000").await
}
