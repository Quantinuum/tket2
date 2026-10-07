//! Compatibility shims for dependencies using obsolete Emscripten symbols.

unsafe extern "C" {
    fn emscripten_get_now() -> f64;
}

/// Forwards `instant 0.1`'s obsolete clock import to Emscripten's current symbol.
///
/// `instant` imports `_emscripten_get_now`, but modern Emscripten main modules
/// expose `emscripten_get_now` to dynamically linked side modules.
///
/// Should be removed once `ascent` replaces its unmaintained `instant` dependency with `web-time`.
/// See <https://github.com/s-arash/ascent/pull/75> and <https://github.com/s-arash/ascent/issues/87>.
#[unsafe(no_mangle)]
pub extern "C" fn _emscripten_get_now() -> f64 {
    // SAFETY: `emscripten_get_now` takes no arguments and returns a monotonic
    // timestamp in milliseconds, as declared by the Emscripten API.
    unsafe { emscripten_get_now() }
}
