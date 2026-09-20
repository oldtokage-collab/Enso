# Post-build script: produces merge.bin (bootloader + partitions + boot_app0 + firmware)
# next to firmware.bin so the image can be registered on M5Burner.
Import("env")
import os
import subprocess

def merge_bin(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    platform = env.PioPlatform()
    framework_dir = platform.get_package_dir("framework-arduinoespressif32")
    esptool_dir = platform.get_package_dir("tool-esptoolpy")

    firmware   = os.path.join(build_dir, "firmware.bin")
    bootloader = os.path.join(build_dir, "bootloader.bin")
    partitions = os.path.join(build_dir, "partitions.bin")
    boot_app0  = os.path.join(framework_dir, "tools", "partitions", "boot_app0.bin")
    output     = os.path.join(build_dir, "merge.bin")
    esptool    = os.path.join(esptool_dir, "esptool.py")

    cmd = [
        env.subst("$PYTHONEXE"), esptool,
        "--chip", "esp32s3", "merge_bin",
        "-o", output,
        "--flash_mode", "dio",
        "--flash_freq", "80m",
        "--flash_size", "8MB",
        "0x0000", bootloader,
        "0x8000", partitions,
        "0xe000", boot_app0,
        "0x10000", firmware,
    ]
    print("Creating merge.bin ...")
    subprocess.check_call(cmd)
    print("merge.bin written to", output)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", merge_bin)
