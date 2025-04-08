Endret på spawn_rov_tms.launch og start_thruster_manager_tms
spawn filen har vi lagt til to noder, en for å konvertere LaerScan til PointCloud, og en for å konvertere PointCloud til XY koordinater.
lagd en Scripts mappe med tre filer, de to som er nevnt over og en for å publishe til Thrusterene hvilken vei de skal thruste
denne siste noden er lagt til i thruster manager filen
