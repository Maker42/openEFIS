docker run --network autopilot --volume /home/garrett:/home/garrett --name pubsub -dit sensors ash #python3 PubSub.py
docker run --network autopilot --volume /home/garrett:/home/garrett --name eventdb -dit eventdb ash
docker run --network autopilot --volume /home/garrett:/home/garrett --name sensor_input -dit sensors ash #python3 MockRawData.py raw_sensors_mock.yml
docker run --network autopilot --volume /home/garrett:/home/garrett --name sensor_processing -dit sensors ash
