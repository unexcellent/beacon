fn main() {
    embuild::espidf::sysenv::output();

    let img = image::open("examples/patch.png").expect("open examples/patch.png");
    let img = img.resize_exact(320, 240, image::imageops::FilterType::Lanczos3);
    let rgb_bytes = img.to_rgb8().into_raw();

    let out_dir = std::env::var("OUT_DIR").unwrap();
    let out_path = std::path::PathBuf::from(out_dir).join("patch_rgb.bin");
    std::fs::write(&out_path, &rgb_bytes).expect("write patch_rgb.bin");

    println!("cargo:rerun-if-changed=examples/patch.png");
}
