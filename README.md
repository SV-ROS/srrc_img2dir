# srrc_img2dir

brief usage for processing jpg files:

0. check out this repo into your catkin_ws
1. open file launch\mono2dir.launch
2. replace value for parameter 'src_image_src_folder' with path to your folder with jpg files
3. replace value for parameter 'output_folder' with the path you want to output the processed images to (it will be created if it does not exist)
4. call roslaunch srrc_img2dir test_mono2dir.launch and wait.
