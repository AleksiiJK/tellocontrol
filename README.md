Täällä on sitten vain hyvät ja oikeat tiedostot

Useful commands:

Simulator
ros2 launch tello_gazebo simple_launch.py

Tello driver
ros2 launch tello_driver teleop_launch.py

Drone commands
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone1
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'up 30'}"
ros2 launch tello_driver teleop_launch.py 



Kontrolli-idea:
Tsekkaa neliötä keskellä, jos keskellä ei oo yhtään vihreetä ja ulkopuolella on vihreetä, mene x määrä sokkona eteenpäin.

TO DO:
Atte:
Laske läpäistyt portit DONE
Muuta käyttäytymistä porttien mukaan DONE
Oleta, että kytiksen funktio publishaa centroidia ja implementoi et se publishataan
Tee simppeli scripti joka tulee stop-merkin lähelle ja ländää

Kytis:
Tunnista QR-koodi edge detectionissa, punainen a vihreä  DONE
Luo centroidi markkerien avulla ilman että muokkaat originaaliframea.



Eero:
Hallitse kokonaisuutta ja suunnitelmaa kommunikoi + Gameplant


KILPAILUPÄIVÄN SUUNNITELMA
setup:
-Konfiguroi värit: hsv_filter_node ja rqf_reconfigure rqf_reconfigure
-Lisää oikea järjestys porteille, jotta homma toimii. (jos implementoitu)
-Rukoile
launch:
aja detect.launch.py 
aja takeoff + ehkä up

-Rukoile