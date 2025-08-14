fn main() -> std::io::Result<()> {
    println!("cargo::rustc-check-cfg=cfg(context, values(\"simulator\", \"mcu\", \"pico\", \"picow\", \"pico2\", \"pico2w\", \"rp\", \"rp2040\", \"rp2350\"))");

    #[allow(unused)]
    let mut board_config_path: Option<std::path::PathBuf> = None;

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "pico", feature = "picow"))] {
            board_config_path = Some([env!("CARGO_MANIFEST_DIR"), "boards", "rp2040", "config.toml"].iter().collect());
        } else if #[cfg(any(feature = "pico2", feature = "pico2w"))] {
            board_config_path = Some([env!("CARGO_MANIFEST_DIR"), "boards", "rp2350", "config.toml"].iter().collect());
        } else if #[cfg(any(feature = "simulator"))] {
            board_config_path = Some([env!("CARGO_MANIFEST_DIR"), "boards", "simulator", "config.toml"].iter().collect());
        }
    }

    if let Some(board_config_path) = board_config_path {
        let config = std::fs::read_to_string(board_config_path.clone())?;
        let toml = config.parse::<toml::Table>().expect("invalid board config toml");
        for link_arg in
            toml.get("link_args").and_then(toml::Value::as_array).into_iter().flatten()
        {
            if let Some(option) = link_arg.as_str() {
                println!("cargo:rustc-link-arg={option}");
            }
        }
        for link_search_path in
            toml.get("link_search_path").and_then(toml::Value::as_array).into_iter().flatten()
        {
            if let Some(mut path) = link_search_path.as_str().map(std::path::PathBuf::from) {
                if path.is_relative() {
                    path = board_config_path.parent().unwrap().join(path);
                }
                println!("cargo:rustc-link-search={}", path.to_string_lossy());
            }
        }
        for cfgs in
            toml.get("cfgs").and_then(toml::Value::as_array).into_iter().flatten()
        {
            if let Some(option) = cfgs.as_str() {
                println!("cargo::rustc-cfg={option}");
            }
        }
        println!("cargo:rerun-if-changed={}", board_config_path.display());
    }



    Ok(())
}