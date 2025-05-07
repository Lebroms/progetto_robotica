TASK 1 
==============

Obiettivo:
----------
Il pacchetto "task_1" è stato creato per permettere al robot TIAGo di effettuare una scansione dell’ambiente tramite il movimento della testa e rilevare marker ArUco, trasformando le coordinate dei marker dal frame della camera al frame base del robot.

Struttura del pacchetto:
-------------------------
Workspace: tiago_public_ws
Percorso: tiago_public_ws/src/task_1/

Contenuto:
- action_client.py: Client che invia il goal per avviare la scansione.
- action_server.py: Server che gestisce il movimento della testa del robot.
- __init__.py: Necessario per il riconoscimento del pacchetto come modulo Python.
- launch/: contiene il launch file per eseguire client e server insieme --> action_app.launch.py.

Action personalizzata:
-----------------------
L’interfaccia per l’action è definita nel pacchetto "personal_interfaces".
L’action consente al client di specificare i parametri di scansione (Goal: left_limit, right_limit, step_angle; Result: success, Feedback: current_angle), e al server di effettuare il movimento e inviare feedback/richiesta completata.

Funzionalità implementate [DA AGGIORNARE]:
--------------------------
- Avvio dell’action server
- Invio del goal tramite client
- Movimento incrementale della testa del robot
- Nodo per la rilevazione dei marker ArUco tramite OpenCV
- Stampa della posa dei marker in quaternioni
- (In fase di sviluppo) trasformazione della posa nel frame base del robot
- (Da completare) visualizzazione in RViz e test in ambiente simulato Gazebo

Dipendenze [DA AGGIORNARE]:
-----------
- ROS 2 (es. Humble)
- rclpy
- tf2_ros
- geometry_msgs
- sensor_msgs
- OpenCV (cv2 e cv2.aruco)
- action_msgs
- launch, launch_ros

Esecuzione:
-----------
1. Source del workspace:
   source ~/tiago_public_ws/install/setup.bash

2. Lancio del pacchetto (se presente il file launch):
   ros2 launch task_1 action_app.launch.py

Stato del progetto:
-------------------
[X] Action server
[X] Action client
[X] Nodo ArUco con stampa pose
[ ] Trasformazione frame base (tf2_ros)
[ ] Visualizzazione RViz
[ ] Test in Gazebo

Autori:
-------
- Barnaba Gianluca
- Bini Federico
- Bussone Beatrice
- Principi Federico
- Scorza Luca

