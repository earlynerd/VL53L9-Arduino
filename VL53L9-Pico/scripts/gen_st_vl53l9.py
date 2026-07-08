from pathlib import Path
import shutil

Import("env")


ST_FILES = (
    "LICENSE.txt",
    "vl53l9.c",
    "vl53l9.h",
    "vl53l9_patch.h",
    "vl53l9_platform.h",
    "vl53l9_reg.h",
)


def _option_as_text(name, default=""):
    value = env.GetProjectOption(name, default)
    if isinstance(value, (list, tuple)):
        return " ".join(str(item) for item in value)
    return str(value)


def _as_define_text(value):
    if isinstance(value, (list, tuple)):
        return "=".join(str(item) for item in value)
    return str(value)


def _st_driver_enabled():
    if env.subst("$PIOENV") == "rp2350_st_driver":
        return True

    build_flags = _option_as_text("build_flags")
    cpp_defines = " ".join(_as_define_text(item) for item in env.get("CPPDEFINES", []))
    return (
        "-DVL53L9_ENABLE_ST_DRIVER=1" in build_flags
        or "VL53L9_ENABLE_ST_DRIVER=1" in build_flags
        or "VL53L9_ENABLE_ST_DRIVER=1" in cpp_defines
    )


if _st_driver_enabled():
    project_dir = Path(env.subst("$PROJECT_DIR"))
    build_dir = Path(env.subst("$BUILD_DIR"))
    source_option = _option_as_text("custom_vl53l9_st_source")
    source_dir = Path(source_option)
    if not source_dir.is_absolute():
        source_dir = (project_dir / source_dir).resolve()

    missing = [name for name in ST_FILES if not (source_dir / name).exists()]
    if missing:
        missing_list = ", ".join(missing)
        raise RuntimeError(
            "VL53L9 ST driver build is enabled, but the ST BSP component files "
            f"were not found in {source_dir}. Missing: {missing_list}. "
            "Download X-CUBE-53L9A1 from ST or update custom_vl53l9_st_source."
        )

    generated_dir = build_dir / "generated" / "vl53l9_st"
    generated_dir.mkdir(parents=True, exist_ok=True)

    for name in ST_FILES:
        src = source_dir / name
        dst = generated_dir / name
        if not dst.exists() or dst.stat().st_mtime < src.stat().st_mtime:
            shutil.copy2(src, dst)

    env.Append(CPPPATH=[str(generated_dir)])
    env.BuildSources(
        str(build_dir / "vl53l9_st"),
        str(generated_dir),
        src_filter=["+<vl53l9.c>"],
    )
