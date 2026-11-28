# Slicer profile fixtures (AM-10)

Small hand-written files in the layouts the slicers use, holding only the values the tests in
`crates/amg-am/tests/import.rs` read: sizes, nozzle and layer values, preset names and
`inherits`. No G-code, no names, e-mail addresses, serial numbers or tokens.

| Folder | Layout | Values |
|---|---|---|
| `orca/` | data folder (`user/`, `system/`) and install folder (`resources/profiles/`) | public Ender-3 V3 KE preset values; the user preset with the 220 x 220 x 240 override of S16 note §6.1 |
| `orca-narrow/` | a user preset only | the same preset with the 200 x 220 bed it has since (2026-11-28) |
| `chitubox/` | `MachineConfig` folder | public ELEGOO Mars 3 Pro values |
| `prusa/` | data folder (`vendor/`, `printer/`) | invented |
