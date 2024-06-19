sudo chmod a+rw /dev/ttyUSB0
python3 init_param.py &

# 等待3秒
sleep 3

python3 erhehe.py &
python3 erfanfan.py &
python3 send_param.py &
python3 position1.py &
python3 fan.py &
python3 action_gpt.py &

