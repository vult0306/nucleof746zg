use std::{
    env,
    io::{self},
};

fn main() -> Result<(), Error> {
    println!("cargo:rerun-if-changed=build.rs");
    Ok(())
}

#[derive(Debug)]
enum Error {
    Env(env::VarError),
    Io(io::Error),
}

impl From<env::VarError> for Error {
    fn from(error: env::VarError) -> Self {
        Self::Env(error)
    }
}

impl From<io::Error> for Error {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}