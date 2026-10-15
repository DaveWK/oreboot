use std::{env, process, str::FromStr};

use clap::{Args, Parser, Subcommand};
use clap_verbosity_flag::{InfoLevel, Verbosity};
use log::{error, info};

mod gdb_detect;
mod target;
mod util;

mod qemu;
mod spacemit;
mod starfive;
mod sunxi;

#[derive(Parser)]
#[clap(name = "xtask")]
#[clap(about = "The oreboot build system", long_about = None)]
struct Cli {
    #[clap(subcommand)]
    command: Commands,
    #[clap(flatten)]
    env: Env,
    #[clap(flatten)]
    verbose: Verbosity<InfoLevel>,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Make this project, i.e., build a full image
    Make,
    /// Build image and flash to target
    Flash,
    /// Run without flashing, as supported by the platform
    Run,
    /// Analyze stack sizes and other properties
    Analyze,
    /// View assembly code, as in objdump
    Asm,
    /// Debug code using gdb, as supported by the platform
    Gdb,
    /// Pack a local payload with the published K1 development keys; no device IO
    K1xPack {
        input: std::path::PathBuf,
        output: std::path::PathBuf,
    },
    /// Derive or check the intentionally public K1 development fixtures
    K1xDevKeys {
        /// Compare the derived keys with the committed ones instead
        #[clap(long)]
        check: bool,
        /// Alternate directory for disposable public development-key checks
        #[clap(long)]
        directory: Option<std::path::PathBuf>,
    },
}

#[derive(Clone)]
enum Memory {
    /// Operate on NAND flash
    Nand,
    /// Operate on NOR flash
    Nor,
}

impl FromStr for Memory {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "nand" => Ok(Self::Nand),
            "nor" => Ok(Self::Nor),
            others => Err(format!("unknown memory type {others}")),
        }
    }
}

#[derive(Clone)]
enum DramSize {
    TwoG,
    FourG,
    EightG,
}

impl FromStr for DramSize {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "2g" => Ok(Self::TwoG),
            "4g" => Ok(Self::FourG),
            "8g" => Ok(Self::EightG),
            others => Err(format!("unknown DRAM size {others}")),
        }
    }
}

#[derive(Args)]
struct Env {
    #[clap(
        long = "release",
        global = true,
        help = "Build in release mode",
        long_help = None,
    )]
    release: bool,
    #[clap(
        long = "supervisor",
        global = true,
        help = "Build with arch-specific supervisor support",
        long_help = None,
    )]
    supervisor: bool,
    #[clap(long, global = true, help = "Mainboard to build")]
    mainboard: Option<String>,
    #[clap(long, global = true, help = "Board variant")]
    variant: Option<String>,
    #[clap(long, global = true, help = "Target memory description")]
    memory: Option<Memory>,
    #[clap(long, global = true, help = "DRAM size")]
    dram_size: Option<DramSize>,
    #[clap(long, global = true, help = "Path to raw payload")]
    payload: Option<String>,
    #[clap(long, global = true, help = "Path to dtb")]
    dtb: Option<String>,
}

fn main() {
    let args = Cli::parse();
    env_logger::Builder::new()
        .filter_level(args.verbose.log_level_filter())
        .init();
    info!("=== oreboot build system ===");
    match &args.command {
        Commands::K1xDevKeys { check, directory } => {
            spacemit::k1x_dev_keys(*check, directory.clone());
            return;
        }
        Commands::K1xPack { input, output } => {
            if let Err(e) = spacemit::k1x_pack(input, output) {
                error!("K1 development pack failed: {e}");
                process::exit(1);
            }
            return;
        }
        _ => {}
    }
    let cur_path = env::current_dir().unwrap();
    let target = if let Some(target) = target::parse_target(
        &cur_path,
        args.env.mainboard.as_deref(),
        args.env.variant.as_deref(),
    ) {
        target
    } else {
        error!(
            "can't decide target for task
    Change directory to mainboard and run again, or
    use `--mainboard <VENDOR/BOARD>` and `--variant <VARIANT>` when necessary."
        );
        process::exit(1)
    };
    target.execute_command(&args);
}
