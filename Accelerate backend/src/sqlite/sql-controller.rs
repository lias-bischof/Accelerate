use crate::kinematics::Joint;
use rusqlite::{params, Connection, Error, Result};

pub struct SQLController {
    connection: Connection
}

impl SQLController {

    pub fn create(path: &str) -> Result<Self> {
        let connection = Connection::open(path)?;
        connection.execute(
            "CREATE TABLE IF NOT EXISTS joints (
                id INTEGER PRIMARY KEY CHECK (id BETWEEN 0 AND 5),
                a REAL NOT NULL,
                d REAL NOT NULL,
                α REAL NOT NULL,
                Δϑ REAL NOT NULL,
                θmin REAL NOT NULL,
                θmax REAL NOT NULL
            )", []
        )?;
        Ok(Self { connection })
    }

    pub fn save(&mut self, joints: &[Joint; 6]) -> Result<()> {
        let transaction = self.connection.transaction()?;
        {
            let mut statement = transaction.prepare(
                "INSERT INTO joints (id, a, d, α, Δϑ, θmin, θmax)
                 VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7)
                 ON CONFLICT(id) DO UPDATE SET
                    a = excluded.a,
                    d = excluded.d,
                    α = excluded.α,
                    Δϑ = excluded.Δϑ,
                    θmin = excluded.θmin,
                    θmax = excluded.θmax",
            )?;
            for i in 0..6 {
                let joint = &joints[i];
                statement.execute(params![
                    i as i64,
                    joint.a,
                    joint.d,
                    joint.α,
                    joint.Δϑ,
                    joint.θmin,
                    joint.θmax,
                ])?;
            }
        }
        return transaction.commit()
    }

    pub fn load(&self) -> Result<[Joint; 6]> {
        let mut statement = self.connection.prepare(
            "SELECT a, d, α, Δϑ, θmin, θmax FROM joints ORDER BY id"
        )?;
        let joints = statement.query_map([], |row| {
            Ok(Joint {
                a: row.get(0)?,
                d: row.get(1)?,
                α: row.get(2)?,
                Δϑ: row.get(3)?,
                θmin: row.get(4)?,
                θmax: row.get(5)?,
            })
        })?.take(6).collect::<Result<Vec<_>>>()?;
        joints.try_into().map_err(|_| Error::QueryReturnedNoRows)
    }
}
