#
# download_mks_assets.py
# Added by HAS_TFT_LVGL_UI to download assets from Makerbase repo
#
import pioutil
if pioutil.is_pio_build():
    import requests, zipfile, tempfile, shutil
    from pathlib import Path

    env = pioutil.env
    url = "https://github.com/makerbase-mks/Mks-Robin-Nano-Marlin2.0-Firmware/archive/0263cdaccf.zip"
    deps_path = Path(env.Dictionary("PROJECT_LIBDEPS_DIR"))
    zip_path = deps_path / "mks-assets.zip"
    assets_path = Path(env.Dictionary("PROJECT_BUILD_DIR"), env.Dictionary("PIOENV"), "assets")

    def download_mks_assets():
        print("Downloading MKS Assets for TFT_LVGL_UI")
        r = requests.get(url, stream=True)
        # the user may have a very clean workspace,
        # so create the PROJECT_LIBDEPS_DIR directory if not exits
        if not deps_path.exists():
            deps_path.mkdir()
        with zip_path.open('wb') as fd:
            for chunk in r.iter_content(chunk_size=128):
                fd.write(chunk)

    def copy_mks_assets():
        print("Copying MKS Assets for TFT_LVGL_UI")
        output_path = Path(tempfile.mkdtemp())
        zip_obj = zipfile.ZipFile(zip_path, 'r')
        zip_obj.extractall(output_path)
        zip_obj.close()
        if assets_path.exists() and not assets_path.is_dir():
            assets_path.unlink()
        if not assets_path.exists():
            assets_path.mkdir()
        base_path = ''
        for filename in output_path.iterdir():
            base_path = filename
        fw_path = (output_path / base_path / 'Firmware')
        font_path = fw_path / 'mks_font'
        for filename in font_path.iterdir():
            shutil.copy(font_path / filename, assets_path)
        pic_path = fw_path / 'mks_pic'
        for filename in pic_path.iterdir():
            shutil.copy(pic_path / filename, assets_path)
        shutil.rmtree(output_path, ignore_errors=True)

    if not zip_path.exists():
        download_mks_assets()

    if not assets_path.exists():
        copy_mks_assets()
