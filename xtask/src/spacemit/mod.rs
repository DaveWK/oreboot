use std::path::PathBuf;

use super::Cli;

mod k1x;
mod k1x_hdr;
mod k1x_keys;

pub fn k1x_dev_keys(check: bool, directory: Option<PathBuf>) {
    k1x_keys::run(check, directory);
}

#[derive(Debug, Clone, Copy)]
pub enum Board {
    K1,
}

impl Board {
    pub(crate) fn execute_command(self, command: &Cli, dir: &PathBuf, features: Vec<String>) {
        match self {
            Board::K1 => k1x::execute_command(command, dir, features),
        }
    }
}

/// Offline deterministic development-image packing. Never overwrites an output.
pub fn k1x_pack(input: &std::path::Path, output: &std::path::Path) -> std::io::Result<()> {
    use std::io::Write;
    let bytes = std::fs::read(input)?;
    let image = k1x_hdr::build(&bytes);
    let mut file = std::fs::File::options()
        .write(true)
        .create_new(true)
        .open(output)?;
    file.write_all(&image)
}
