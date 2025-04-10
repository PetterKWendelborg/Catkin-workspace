
Update 10.04.2025

La til vertical beams i sonar snippers filen. Bare ta ctrl+s og søk på vertical i sonar_snipper.xacro så finner du det som er lagt til.

Forandret hvor kameraet spawner i vannverdenen vi bruker som heter ocean_waves, dere kan forandre start posisjonen med koden nederst i filen 
    <camera name='user_camera'>
        <pose>5 8 -8 0 0 -1.57</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>


Fikset på hydrodynamikken i TMS filen da damping var dobbelt opp. 