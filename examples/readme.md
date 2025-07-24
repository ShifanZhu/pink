In terminal one
python sub_and_visualize_meshcat_adjusted_urdf.py

In terminal two
python steven_lcm_publisher.py

You can change the   "time.sleep(1111110.033)"   to stop and visualize the frames, the terminal one also prints target/actual frame positions



Todo: 
1. Check the visualization to see if you need to make the specific link length longer or shorter, then modify human-model-generator length scale for each joint.
2. Regenerate human-mode in terminal three: cd human-model-generator/code, python main.py, type the name (e.g. male)
3. Run above two commands to see if the visualization gets better.
4. uncomment one more link (e.g. ELBOW_LEFT) and modify one by one. (I sugget go from base to further links. I have roughly modified Bottom/Mid/TopLumbar/Neck), you can continue with CLAVICLE_LEFT.
5. You can check the link association based on the figure in slides, which is Kinect's detection results, and URDF.: https://docs.google.com/presentation/d/1RBqr5ySCMIjF1a1gSDwaLtCkPLexApeccypdHQexo-M/edit?slide=id.g36c8bfa5891_0_13#slide=id.g36c8bfa5891_0_13

