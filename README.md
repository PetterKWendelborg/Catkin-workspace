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
