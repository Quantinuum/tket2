import argparse
import io
import json
import platform
import posixpath
import tarfile
import urllib.request
from pathlib import Path

parser = argparse.ArgumentParser(
    description="Fetch and extract hugrverse-env llvm/tket artifacts for the current platform."
)
parser.add_argument(
    "install_path",
    nargs="?",
    default="./target/hugrenv/",
    help="Destination directory for extracted files (default: ./target/hugrenv/).",
)
args = parser.parse_args()

install_path = Path(args.install_path).expanduser().resolve()
install_path.mkdir(parents=True, exist_ok=True)
script_dir = Path(__file__).resolve().parent
lock_path = script_dir.parent / "hugrenv.lock"
lock = json.loads(lock_path.read_text(encoding="utf-8"))
version = lock["version"]

os_name = platform.system().lower()
arch = platform.machine().lower()
if os_name == "linux":
    platform_key = "manylinux_2_28"
    arch_key = {
        "x86_64": "x86_64",
        "amd64": "x86_64",
        "aarch64": "aarch64",
        "arm64": "aarch64",
    }.get(arch)
elif os_name == "darwin":
    platform_key = "macosx_11_0"
    arch_key = {
        "x86_64": "x86_64",
        "amd64": "x86_64",
        "aarch64": "aarch64",
        "arm64": "aarch64",
    }.get(arch)
elif os_name == "windows":
    platform_key = "win"
    arch_key = {"x86_64": "amd64", "amd64": "amd64"}.get(arch)
else:
    raise SystemExit(f"Unsupported platform: os={os_name} arch={arch}")

for package in ("llvm", "tket"):
    try:
        lock["hashes"][platform_key][arch_key][package]
    except KeyError as err:
        raise SystemExit(
            f"Unsupported hugrenv target in lockfile: platform={platform_key} arch={arch_key} package={package}"
        ) from err
    target = f"{platform_key}_{arch_key}"
    url = f"https://github.com/Quantinuum/hugrverse-env/releases/download/v{version}/hugrenv-{package}-{target}.tar.gz"
    print(f"Downloading {package} from {url}")
    data = urllib.request.urlopen(url).read()
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as tar:
        members = []
        for member in tar.getmembers():
            stripped = member.name.split("/", 1)
            if len(stripped) == 1:
                continue
            relative_name = posixpath.normpath(stripped[1])
            if (
                not relative_name
                or relative_name == "."
                or relative_name.startswith("../")
                or relative_name.startswith("/")
            ):
                continue
            member.name = relative_name
            members.append(member)
        tar.extractall(path=install_path, members=members)

print("")
print(f"hugrenv {version} installed in {install_path}")
print(
    "To use the hugrenv libraries, set the following environment variables in your shell."
)
print("")
if os_name == "windows":
    p = str(install_path)
    print("PowerShell:")
    print(f'$env:TKET_C_API_PATH = "{p}"')
    print(f'$env:LLVM_SYS_211_PREFIX = "{p}"')
    print(f'$env:LIBCLANG_PATH = "{p}\\\\lib"')
    print(f'$env:PATH = "{p}\\\\bin;{p}\\\\lib;{p}\\\\lib64;$env:PATH"')
else:
    p = str(install_path)
    print("Bash/Zsh:")
    print(f'export TKET_C_API_PATH="{p}"')
    print(f'export LLVM_SYS_211_PREFIX="{p}"')
    print(f'export LIBCLANG_PATH="{p}/lib"')
    print(f'export PATH="{p}/bin:$PATH"')
    if os_name == "darwin":
        print(f'export DYLD_LIBRARY_PATH="{p}/lib:{p}/lib64:$DYLD_LIBRARY_PATH"')
    else:
        print(f'export LD_LIBRARY_PATH="{p}/lib:{p}/lib64:$LD_LIBRARY_PATH"')
