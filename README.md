# ESP32 Rust Demo

A very simple demo of programming an ESP32 in Rust

## Commands

See connected serial devices, you should see something like
`/dev/cu.usbserial-110`

```shell
ls /dev/tyy.*
```

Run the project with dev profile on the board

```shell
cargo run
```

Run the project in release on the board

```shell
cargo run --release
```
