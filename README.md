Lagt på thruster på ROV i riktig retning, og lagd thruster manager for den.
To nye mapper:
- Rov_control,
  Her ligger rov sine thruster filer
- tms_control,
  Her ligger tms sine thruster filer
Har også lagt til sonar_snippets og thruster_snippets inn under urdf mappen slik at endringer vi gjør på thruster modulen og soaneren er lett overførlig til github.
Fikset slik at tms hydrodynamikken ikke lenger er dobbelt opp.
I rov.xacro
- slettet første versjon av thrustere fra filen
- byttet alle size_x til size_y og omvendt slik at x aksen peker riktig vei
Thrusterne er nå scaleable

Mapper og filer som nå ikke blir brukt, men som jeg ikke vet hvordan å fjerne fra Github
- deep_ocean/config, er blitt erstatet av rov_control og tms_control
- deep_ocean/launch/alle launch filer inni her som ikke er spawn_rov_tma.launch, start_thruster_manager filene er flyttet til rov_control og tms_control
- deep_ocean/urdf/kladd.xacro og box_rov.xacro, disse blir ikke brukt noen plasser.

V2

Lagt til flere scripts, slik at ROV også kan headde
Også flere scripts fra Johnny sin branch slik at TMS har en 3D sonar + scripts som lar den fungere
ArUco mesh er lagt til i meshes og i TMS.xacro, både .png og .dae
Sonar lagt til på ROV
Endret mass på sonar og thrusters til veldig lite
IMU er lagt til TMS + graf på slutten av simulering som viser yaw
Fjernet ubrukte mapper og filer som nevnt over

La til en Materials mappe, her ligger en scripts mappe for python scripts for å lage calibrationbrett og ArUco markør
og textures mappe som inneholder Aruco og checkerboard pnd og dae filer
IMU på ROV
La til ArUco markør og calibration brett i tms og tms_second.xacro


Lagd ny mappe som heter robots der tms-ene og rov-en ligger, endret launch filen slik at det fungere med de nye plasseringene
la til 4 nye 4x4 ArUco markører i materials/textures, men de er ikke i bruk enda

v3
- Filer jeg har lagt til:

rov_threed.xacro

pointcloud_data_conv_rov_threed.py

rov_controller_threed.py

rov_approach_threed.py

rov_angular_velocity_z.py

rov_linear_acceleration_x.py

rov_controller_threed_advanced.py

rov_approach_threed_advanced.py

rov_angle_and__ang_vel_threed.py

- Filer jeg har endret navn på:

tms_second.xacro til tms_threed.xacro

pcdata_conv_second.py -> pointcloud_data_conv_tms_threed.py

rov_approach.py -> rov_approach_twod.py

rov_controller.py -> rov_controller_twod.py

time_and_angular_velocity_z_values.py -> tms_angular_velocity_z.py

tms_controller.py -> tms_controller_twod.py

tms_controller_second.py -> tms_controller_threed.py

tms_controller_second_new.py -> tms_controller_threed_advanced.py

- Andre ting jeg har gjort:

lagt til en ny mappe som heter calibration_board_pictures inn i materials -> scripts
Der ligger det aruco kaibreingsbrett bilder i simulasjons verdenen og koden for å lage kalibrerings verdiene

lot merke til at rov_controller_twod.py manglet "last_time_in_window = None" under hver elif statement, så jeg la dem til.

oppdaterte tms controller med mer avansert hysterisis window slik at den nå passer sammen med de andre løsningne med boolean variabel og timer

har endret navn fra tms_second.xacro til tms_threed.xacro, og har derfor endret tilsvarende navn i spawn_rov_tms.launch.

har endret på spawn_rov_tms.launch ved å legge til flere noder som tilpasser 3d sonar, samme med launch filene for rov/tms thruster_manager filene

hadde gjort noen feil med navnene til tms_threed.xacro iforhold til depth camera, haddet tidligere fjernet $(arg namespace)/ fra noen av linksa.
det førte til at jeg ikke kunne visualisere pointcloud. (burde egentlig gjøres for alle links....)

har lagt til depth_camera(etterligning av 3d sonar) til rov_threed.xacro 


- Et par tankeganger:
Burde nok kjøre plotting scripts seperat slik at den ikke plotter under hele simulasjons perioden, blir en ganske kompremert plot

Eneste forskjellen mellom 3d sonar og 2d sonar i forhold til pointcloud_data_conv_rov.py er egentlig bare hvem de subscriber til, 
så trenger egentlig ikke 4, men bare 2 også bare bytte subscriber i filen om man skal bytte sonar.

