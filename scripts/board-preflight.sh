#!/bin/sh
# Read-only diagnostics: never open/configure the UART or issue a motor command.
set -u
role= binary= uart= sdk=
usage() {
    echo "sh board-preflight.sh --role eff|obs --binary /path/mfs_eff --uart /dev/ttyPS1 [--sdk-binary /path/libxtsdk_shared.so]"
}
while [ "$#" -gt 0 ]; do
    case "$1" in
        --help|-h) usage; exit 0 ;;
        --role|--binary|--uart|--sdk-binary)
            [ "$#" -ge 2 ] || { usage; exit 2; }
            case "$1" in
                --role) role=$2 ;; --binary) binary=$2 ;; --uart) uart=$2 ;; --sdk-binary) sdk=$2 ;;
            esac
            shift 2 ;;
        *) usage; exit 2 ;;
    esac
done
case "$role" in eff|obs) ;; *) usage; exit 2 ;; esac
[ -n "$binary" ] && [ -n "$uart" ] || { usage; exit 2; }
failures=0
fail() { echo "FAIL: $*"; failures=$((failures + 1)); }
echo "Board: $(uname -m) / $(getconf LONG_BIT 2>/dev/null || echo unknown)-bit"
if [ -x "$binary" ]; then
    if command -v file >/dev/null 2>&1; then file "$binary"; fi
    if command -v ldd >/dev/null 2>&1; then
        deps=$(ldd "$binary" 2>&1); echo "$deps"
        case "$deps" in *"not found"*) fail "missing runtime libraries" ;; esac
    fi
    "$binary" --help >/dev/null 2>&1 || fail "binary cannot execute (architecture/loader/ABI)"
else fail "binary missing or not executable: $binary"; fi
if [ -c "$uart" ] && [ -r "$uart" ] && [ -w "$uart" ]; then
    echo "PASS: UART device and permissions: $uart"
else fail "UART path/type/permissions: $uart"; fi
if [ "$role" = obs ]; then
    if [ -n "$sdk" ] && [ -r "$sdk" ]; then
        if command -v file >/dev/null 2>&1; then
            desc=$(file -b "$sdk"); echo "$desc"
            bits=$(getconf LONG_BIT 2>/dev/null || echo unknown)
            case "$bits:$desc" in 32:*"ELF 64-bit"*) fail "64-bit SDK cannot run on this 32-bit board" ;; esac
        fi
        if command -v ldd >/dev/null 2>&1; then
            deps=$(ldd "$sdk" 2>&1); echo "$deps"
            case "$deps" in *"not found"*) fail "SDK runtime libraries missing" ;; esac
        fi
    else fail "OBS requires a readable manufacturer SDK binary"; fi
fi
echo "This does not test motor movement, physical calibration, sensor capture or timestamp evidence."
[ "$failures" -eq 0 ] || exit 1
echo "PASS: basic executable/UART prerequisites; proceed with documented hardware checks."
