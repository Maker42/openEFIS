docker run --network autopilot --volume /home/openEFIS:/home/openEFIS --name pubsub -dt sensors python3 PubSub.py
docker run --network autopilot --volume /home/openEFIS:/home/openEFIS --name eventdb -dt eventdb ./startDB.sh
docker run --network autopilot --volume /home/openEFIS:/home/openEFIS --name sensor_processing -dt sensors python3 RunMicroServices.py
