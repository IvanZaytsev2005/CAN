Запуск can утилиты:
```bash
sudo slcand -o -c -s6 /dev/ttyACM0 can0
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000

candump can0 
```
Запуск Vesc Tool:
```bash
flatpak run com.vesc_project.VescTool
```

