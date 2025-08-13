fn main() -> std::io::Result<()> {
    #[allow(unused)]
    let mut board_config_path: Option<std::path::PathBuf> = None;

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "pico", feature = "picow"))] {
            board_config_path = Some([env!("CARGO_MANIFEST_DIR"), "boards", "rp2040", "config.toml"].iter().collect());
        } else if #[cfg(any(feature = "pico2", feature = "pico2w"))] {
            board_config_path = Some([env!("CARGO_MANIFEST_DIR"), "boards", "rp2350", "config.toml"].iter().collect());
        }
    }

    if let Some(path) = board_config_path {
        println!("cargo:BOARD_CONFIG_PATH={}", path.display())
    }

    Ok(())
}