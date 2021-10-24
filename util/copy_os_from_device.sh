trap "kill 0" EXIT

openocd -f interface/stlink.cfg -f target/stm32f1x.cfg&

gdb -ex 'target remote localhost:3333' -ex 'dump ihex memory ./os.syx 0x08006000 0x08033fff' -ex 'set confirm off' -ex 'quit'
