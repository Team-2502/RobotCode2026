// clippy doesn't like that there is auto.rs in /auto, this tells it to shush
#[allow(clippy::module_inception)]
pub mod auto;
pub mod path;
