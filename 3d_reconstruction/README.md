                        3D RECONSTRUCTION EXCERSISE
                        ===========================
In this practice, the intention is to program the necessary logic to allow kobuki
robot to generate a 3D reconstruction of the scene that it is receiving throughout its
left and right cameras.

////////////////////////////////////////////////////////////////////////////////
                        P R E P A R A T I O N
////////////////////////////////////////////////////////////////////////////////
Follow these simple steps:

1. copy the new interface:
```
sudo cp interface/visualization_ice.py /opt/jderobot/lib/python2.7
```

2.Prepare 3d viewer:
```
sudo apt-get install nodejs-legacy
sudo apt-get install npm
cd 3DVizWeb
npm install electron --save-dev --save-exact
npm install jquery
npm install js-yaml
```
////////////////////////////////////////////////////////////////////////////////
                           E X E C U T I O N
////////////////////////////////////////////////////////////////////////////////

Follow these simple steps to launch the practice:

1. First of all, run Gazebo simulator:
    * Execution without seeing the world:
`$ gzserver reconstruccion3D.world`
    * Normal execution (seeing the world):
`$ gazebo reconstruccion3D.world`

2. Then, run the 3d_reconstruction component:
`$ python2 3d_reconstruction.py 3d_reconstruction_conf.yml`

 3. Finally, launch 3D viewer tool:
`$ cd 3DVizWeb/ && npm start`


* To simplify the closure of the environment, just close the Autopark window (s).
  Ctrl + C will give problems.

* To change the configuration of 3DVizWeb:
  ```
    Open 3DVizWeb/public/config.yml
    Modify only the next fields (updatePoints, updateSegments, linewidth, pointsize, camera)
  ```
