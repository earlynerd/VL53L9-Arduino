from pathlib import Path
import subprocess
import sys

Import("env")


def _pioasm_path():
    package_names = (
        "tool-pioasm-rp2040-earlephilhower",
        "tool-pioasm-rp2040",
    )

    for package_name in package_names:
        package_dir = env.PioPlatform().get_package_dir(package_name)
        if package_dir:
            exe_name = "pioasm.exe" if sys.platform.startswith("win") else "pioasm"
            candidate = Path(package_dir) / exe_name
            if candidate.exists():
                return candidate

    raise RuntimeError("pioasm tool package was not found")


project_dir = Path(env.subst("$PROJECT_DIR"))
build_dir = Path(env.subst("$BUILD_DIR"))

pio_source = project_dir / "lib" / "i3c" / "i3c.pio"
generated_dir = build_dir / "generated" / "i3c"
generated_header = generated_dir / "i3c.pio.h"

generated_dir.mkdir(parents=True, exist_ok=True)
env.Append(CPPPATH=[str(generated_dir)])

if (
    not generated_header.exists()
    or generated_header.stat().st_mtime < pio_source.stat().st_mtime
):
    subprocess.check_call(
        [
            str(_pioasm_path()),
            "-o",
            "c-sdk",
            str(pio_source),
            str(generated_header),
        ]
    )
