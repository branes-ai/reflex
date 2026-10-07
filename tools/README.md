# tools/

Host-side developer tools: simulation drivers, gain-tuning and step-response
utilities, log and trace inspectors. Tools may use heavier dependencies than the
header-only library (plotting, file I/O), but nothing here is a dependency of
`branes::reflex`.

Each tool will get a `CMakeLists.txt` entry when the first one lands; the
directory is not yet part of the build.
