# Fog Screen Robot System Assembly

The purpose of this document is to provide step-by-step assembly instructions to assemble a fog screen robot system with Fetch. This document also includes links to 3D printing resources and the code for the controller.

<details>
  <summary>Tip: How to find Table of Contents of this document</summary>
  
  According to [GitHub Docs]([url](https://docs.github.com/en/get-started/writing-on-github/getting-started-with-writing-and-formatting-on-github/basic-writing-and-formatting-syntax#headings)), GitHub automatically generates a table of contents that you can access by clicking the list icon within the file header. Each heading title is listed in the table of contents and you can click a title to navigate to the selected section.
  See below for an example:
  
  ![image](https://github.com/user-attachments/assets/93f58858-45c7-4bf9-ad09-0ee24b585a98)
</details>

## Table of Figures
- Figure 1 Parts List  
- Figure 2 Screw Layout  
- Figure 3 Fog Screen Housing Step 1  
- Figure 4 Fog Screen Housing Step 2  
- Figure 5 Fog Screen Housing Final Step  
- Figure 6 Fan Locations  
- Figure 7 Projector Shelf Legs  
- Figure 8 Projector Shelf Installed  
- Figure 9 Projector Placement  
- Figure 10 Fog Screen Shelf Subassembly  
- Figure 11 Fog Screen Shelf Assembled  
- Figure 12 Fog Machine Bracket Placement-Orientation  
- Figure 13 Fog Machine Assembly  
- Figure 14 Fog Machine Installation  
- Figure 15 3-Pin DMX Port with labels  
- Figure 16 PCB Wiring  
- Figure 17 PCB Install  
- Figure 18 DC 12V to 24V Boost Converter  
- Figure 19 Boost Converter Wired  
- Figure 20 System Wiring  
- Figure 21 Projector Wiring

## Preamble

### Assumptions 
It is assumed that a Fetch Mobile Manipulator is provided, that all parts purchased are in working order and match the dimensions as designed, a 3D printer is utilized to print the 3D parts with Polylactic Acid (PLA), and all M5 hex bolts are installed with the M5 Allen Key that comes with the Fetch robot.

## Procedures
This project will follow a list of instructions supplemented by figures to recreate the system. 

### Step 1
1. Assemble all materials and tools as they are listed in Figure 1 (a csv file is also available in [parts.csv](parts.csv).). Refer to Figure 2 to understand the layout and naming scheme of the screw holes in the Fetch Mobile Manipulator Base.
<div class="image"> <img src="assembly image/Figure 1 Parts List.png" alt=""> </div>
<p style="text-align: center;">Figure 1 Parts List</p>
<div class="image"> <img src="assembly image/Figure 2 Screw Layout.png" alt=""> </div>
<p style="text-align: center;">Figure 2 Screw Layout</p>

### Step 2
2. The *cad_models/iteration_1* folder has the CAD models for 3D printing.  

    Fog machine brackets:
    -	*fog_machine_brackets/fog_machine_back_bracket.STL* for fog machine back bracket.
    -	*fog_machine_brackets/fog_machine_front_bracket.STL* for fog machine front bracket.
    -	*fog_machine_brackets/fog_machine_right_bracket.STL* for fog machine right bracket.
    -	*fog_machine_brackets/fog_machine_left_bracket.STL* for fog machine left bracket.

    Fog screen housing (case) shelf and brackets:
    -	*fog_screen_shelf/fog_screen_shelf.stl* for fog screen housing shelf.
    -	*fog_screen_brackets/fog_screen_on_shelf_bracket_back.STL* for upper back fog screen housing shelf.
    -	*fog_screen_brackets/fog_screen_on_shelf_bracket_left.STL* for upper left fog screen housing shelf.
    -	*fog_screen_brackets/fog_screen_on_shelf_bracket_right.STL* for upper right fog screen housing shelf.

    Projector shelf:
    -	*projector_shelf/projector_shelf_top_with_legs.STL* for upper part of the projector shelf.
    -	*projector_shelf/bottom_leg_2.STL* for back leg of projector shelf.
    -	*projector_shelf/bottom_leg_3.STL* for front leg of projector shelf.
    -	*projector_shelf/bottom_leg_4.STL* for right leg of projector shelf.
    -	*projector_shelf/bottom_leg_1.STL* for left leg of projector shelf.

    The *fog-machine-control* folder has CAD models for controller case.  
    Fog machine controller case:
    -	*case/FogScreenControllerCase.stl* for fog machine controller case.
    -	*case/FogScreenControllerCaseLid.stl* for fog machine controller case lid.

    GitHub repository *TheRARELab/fog-screen-robot system/cad_models/iteration_1/fog_device_model/5mm_housing_lasercut_sheets* has CAD models for fog screen housing. 

    3D printed part:
    -	*5mm_test_airflow_former_iteration1.STL* for half airflow former.

   CAD Files for Foam Boards (or Equivalent Sheet Materials with a Width of 5mm):
    -	*fan_side.STL* for fan side panel.
    -	*front.STL* for external front panel.
    -	*inner_floor.STL* for internal back panel.
    -	*inner_sheet.STL* for internal front panel.
    -	*top.STL* for bottom panel.
    -	*top_with_hole.STL* for top panel.

   All STL files, with expectation of fog screen housing foam boards, should be 3D printed and checked against the model for accuracy.  

### Step 3
3. Components needed:
    -	1 internal back panel (purple) (Laser cut foam or equivalent).
    -	2 internal front panels (orange) (Laser cut foam or equivalent).
    -	1 bottom panel (yellow) (Laser cut foam or equivalent).
    -	Mini Hot Glue Gun Kit & Hot Glue Sticks.

    Install the 3 slots of the internal front panels to the 3 corresponding tabs on both sides of the internal back panel. Stand the assembly upright and insert its tabs into the slots on the bottom panel. Use the Mini Hot Glue Gun Kit & Hot Glue Sticks to attach parts together. Assembly will resemble Figure 3.
<div class="image"> <img src="assembly image/Figure 3 Fog Screen Housing Step 1.png" alt=""> </div>
<p style="text-align: center;">Figure 3 Fog Screen Housing Step 1</p>


### Step 4
4.	Components needed:  
    - 1 top panel (green) (Laser cut foam or equivalent).  
    -	2 external front panels (red) (Laser cut foam or equivalent).  
    -	2 fan side panels (blue) (Laser cut foam or equivalent).  
    -	Mini Hot Glue Gun Kit & Hot Glue Sticks.

    Place the top panel onto the assembly, aligning its slots with the 3 tabs. Attach the 2 external front panels as shown in Figure 4. Repeat the process for the 2 fan side panels. All connections will be hot glued together. 
<div style="display: flex; justify-content: center; align-items: center; gap: 10px; margin-bottom: 10px;">
  <div class="image">
    <img src="assembly image/Figure 4 Fog Screen Housing Step 2 - 1.png" alt="">
    <img src="assembly image/Figure 4 Fog Screen Housing Step 2 - 2.png" alt="">
  </div>
</div>
<p style="text-align: center;">Figure 4 Fog Screen Housing Step 2</p>

### Step 5
5.	Components needed:
    -	2 half airflow formers (white) (3D Printed)
    -	Mini Hot Glue Gun Kit & Hot Glue Sticks.

    For each side of the assembly, slide 1 half airflow former with its hexagon holes area direced to the front. Secure them with hot glue, ensuring a tight seal with no gaps between the foam boards and 3D-printed components and between any foam board panels.The final assembly should resemble Figure 5. 
<div style="display: flex; justify-content: center; align-items: center; gap: 10px; margin-bottom: 10px;">
  <div class="image">
    <img src="assembly image/Figure 5 Fog Screen Housing Final Step - 1.png" alt="">
    <img src="assembly image/Figure 5 Fog Screen Housing Final Step - 2.png" alt="">
  </div>
</div>
<p style="text-align: center;">Figure 5 Fog Screen Housing Final Step</p>

### Step 6
6.	Components needed:
    -	The fog screen housing (Subassembly). 
    -	PWM Fan Hub (2 wires).
    -	6 80 x 80 mm PC Fans.

    Install 6 fans as shown in Figure 6 to the back of the fog screen housing and apply hot glue along the edges and between where the fans meet the Fog Screen Housing, ensuring that the connection is secure. For each fan hub, connect it red to red and black to black to the PWN Fan Hubs. 
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 6 Fan Locations.png" alt="">
  </div>
  <p>Figure 6 Fan Locations</p>
</div>

### Step 7
7.	Components needed:
    -	The front leg of the projector shelf. (3D Printed)
    -	The back leg of the projector shelf. (3D Printed)
    -	The right leg of the projector shelf. (3D Printed)
    -	The left leg of the projector shelf. (3D Printed)
    -	18-8 Stainless Steel Hex Drive Flat Head Screws - 10 mm M5 (4 pieces). 
    -	M5 Allen Key. 

    Secure the projector shelf legs to the base of the Fetch Mobile Manipulator using 4 10mm long M5 screws. The legs will be orientated as shown in Figure 7. Refer to Figure 2 and attach the front leg to E4, the back leg to B1, the right leg to B4, and the left leg to D1. 
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 7 Projector Shelf Legs.png" alt="">
  </div>
  <p>Figure 7 Projector Shelf Legs</p>
</div>

### Step 8
8.	Components needed:
    -	Portable Power Station.
    -	The upper part of the projector shelf. (3D Printed) 
    -	Scissors.

    The Power Station will have its front face facing outward from the Fetch Mobile Manipulator as shown in Figure 8.  
    
    The upper part of the projector shelf should be installed over the Power Station, with each projector shelf leg sliding into the holes in the upper projector shelf. 
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 8 Projector Shelf Installed.png" alt="">
  </div>
  <p>Figure 8 Projector Shelf Installed</p>
</div>

### Step 9
9.	Components needed:
    -	Industrial Strength Velcro Strips. 
    -	Home Theater Projector. 

    Use scissors to cut a 50 x 50mm square piece of Velcro Strips and attach 1 piece to the top of the projector shelf as shown in Figure 8. This piece should be dry fit with the adjusted projector at rotation 20 degrees to the right (counterclockwise), as shown in Figure 9.  
    
    Attach the remaining piece of the Velcro Strips to the bottom of the projector.
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 9 Projector Placement.png" alt="">
  </div>
  <p>Figure 9 Projector Placement</p>
</div>

### Step 10
10.	Components needed:
    -	The fog screen housing shelf (3D Printed).
    -	The upper back fog screen housing bracket (3D Printed).
    -	The upper left fog screen housing bracket (3D Printed).
    -	The upper right fog screen housing bracket (3D Printed).
    -	18-8 Stainless Steel Hex Drive Flat Head Screws - 25 mm M5 (4 pieces).
    -	18-8 Stainless Steel Hex Nut M5 (4 pieces). 
    -	M5 Allen Key.

    Install the upper back fog screen housing bracket into the fog screen housing shelf with 2 25 mm long M5 hex screws, this bracket should be facing into the housing shelf as shown in Figure 10.  
    
    The upper left and right fog screen housing brackets should be installed to the left and right of the upper back fog screen housing bracket with 2 25mm long M5 hex screws, while orientated so the back bracket is in the back. The final assembly will resemble Figure 10. 
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 10 Fog Screen Shelf Subassembly.png" alt="">
  </div>
  <p>Figure 10 Fog Screen Shelf Subassembly</p>
</div>

### Step 11
11.	Components needed:
    -	Fog screen shelf (Subassembly). 
    -	18-8 Stainless Steel Hex Drive Flat Head Screws - 10 mm M5 (4 pieces).
    -	M5 Allen Key.

    Secure the fog screen shelf to the Fetch Mobile Manipulator base using 4 10 mm long M5 screws: attach the two left legs, under the upper back bracket, to C9 and E8; the back leg, under the upper left bracket, to B7; and the right leg, under the upper right bracket, to D6. Install will resemble Figure 11.
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 11 Fog Screen Shelf Assembled.png" alt="">
  </div>
  <p>Figure 11 Fog Screen Shelf Assembled</p>
</div>

### Step 12
12.	Components needed:  
    - The left fog machine bracket (3D Printed).  
    -	The right fog machine bracket (3D Printed).  
    -	The back fog machine bracket (3D Printed).  
    -	18-8 Stainless Steel Hex Drive Flat Head Screws - 16 mm M5 (3 pieces).  
    -	M5 Allen Key.

    Install with 16 mm long M5 screws the left bracket at C6, the right bracket at D9, and the back bracket at A8. They will be orientated as shown in Figure 12. (Note: Do not fully tighten the screws, as the fog machine needs to be properly positioned before being tightened down). 
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 12 Fog Machine Bracket Placement-Orientation.png" alt="">
  </div>
  <p>Figure 12 Fog Machine Bracket Placement / Orientation</p>
</div>

### Step 13
13.	Components needed:
    -	Straight reducer
    -	Flexible pipe - 42mm Diameter, 1m Length. 
    -	Fog machine - 400W.
    -	Hose Clamp - 32mm.
    -	Drill. 
    -	Mini Hot Glue Gun Kit & Hot Glue Sticks.

    Insert the fog machine's output into the smaller end of the straight reducer, secure them it tightly using the hose clamp. To ensure proper fog distribution and prevent forming a vacuum, use drill or equivalent to form several holes on the other end of the straight reducer and the silicon flexible pipe. These components will resemble Figure 13. Connect the other end of the straight reducer into the flexible pipe. Hot glue these connections together.
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 13 Fog Machine Assembly.png" alt="">
  </div>
  <p>Figure 13 Fog Machine Assembly</p>
</div>

### Step 14
14.	Components needed:
    -	Fog machine.
    -	The front fog machine bracket.
    -	18-8 Stainless Steel Hex Drive Flat Head Screws - 16 mm M5 (1 piece).
    -	M5 Allen key.

    Place the fog machine with the installed pipe between the 3 brackets installed in Step 10, ensure the orientation matches Figure 14. Install the front fog machine bracket at F6 using a 16 mm long M5 screw. Adjust the position of the fog machine as needed, then securely tighten the screws on all 4 brackets.
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 14 Fog Machine Installation.png" alt="">
  </div>
  <p>Figure 14 Fog Machine Installation</p>
</div>

### Step 15
15.	Components needed:  
    -	XLR DMX Cable, 3-Pin Male to Female.  
    -	Wire Strippers.  
    -	Multimeter.  
    -	Scissors.  

    Cut the DMX cable’s female port, leaving the male port. Use the wire strippers to strip the jacket of the female inner wires. To connect the wires to the controller first identify them with the multimeter, setting it to continuity (beep) mode: this mode emits a tone when a connection is detected between two wires. When the multimeter beeps it has identified what wire corresponds to what pin. After identifying all of three pins, they will be attached to their corresponding input terminals on the controller. Refer to Figure 15 for pin information.  
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 15 3-Pin DMX Port with labels.png" alt="">
  </div>
  <p>Figure 15 3-Pin DMX Port with labels</p>
</div>

### Step 16
16.	Components needed:
    -	Arduino Nano & Mini USB Cable.
    -	XLR DMX Cable.
    -	PCB.
    -	Scissors.
    -	Screwdriver.

    PCB needs to be placed on a non-conductive surface before connecting the wires to prevent electrical shock. Install the Arduino Nano onto the PCB, all the legs of the Arduino need to fit into the corresponding holes, with the small port facing to the front of the PCB. Connect the small Arduino cable to the Arduino Nano.  
    
    The XLR DMX Cable will connect the output (green) wire to the Output port, the neutral (blue) wire to the Neutral port, and the live (orange) wire to the Live port. Refer to Figure 16 to confirm connections and to Figure 15 to confirm wire type. Use screwdriver to tighten down. 
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 16 PCB Wiring.png" alt="">
  </div>
  <p>Figure 16 PCB Wiring</p>
</div>

### Step 17
17.	Components needed:
    -	PCB.
    -	Fog screen controller case (3D Printed).
    -	Fog screen controller case lid (3D Printed). 
    -	Industrial Strength Velcro Strips.

    Place the PCB into the case with the wires facing and fed through the opening. Slide the lid into place. Hole to see Arduino nano should be placed above nano This will resemble Figure 17.  
    
    Cut 2 50 x 50 mm square of Velcro Strips and attach one piece to the bottom of the fog screen controller case. 
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 17 PCB Install.png" alt="">
  </div>
  <p>Figure 17 PCB Install</p>
</div>

### Step 18
18.	Components needed:
    -	DC 12V to 24V Boost Converter, 3A 72W. 
    -	Wire Connectors (2 pieces). 
    -	DC Power Jack Plug Adapters. 
    -	PWM Fan Splitter Cable (2 wires).
    -	Wire Connector. 
    -	Wire Strippers.
    -	Scissors.
    -	Industrial Strength Velcro Strips.

    Preparing the Fan Splitter Cable:  
    -	Cut the 4-pin male connector using scissors, leaving two long wires of the 4-pin female.
    -	Use wire strippers to remove the jacket of the inner wires, separating the inner red and black wires for later use (can use tape to cover the yellow, blue wires for safety).

    Connecting the DC 12V to 24V Boost Converter Output:
    -	Reference Figure 18 for the positive and negative output cables
    -	For each output wire from the Boost Converter, attach them to 1 wire connector.
    -	Connect the positive (+) output (yellow wire) to the two red inner wires of the Fan Splitter Cable via the wire connector.
    -	Connect the negative (-) output (black wire) to the two black inner wires of the Fan Splitter Cable via the wire connector.

    Connecting the DC 12V to 24V Boost Converter Input:
    -	Reference Figure 18 for the positive and negative output cables
    -	Connect the positive (+) input (red wire) of the boost converter to the positive terminal of the male DC Power Jack Plug Adapters.
    -	Connect the negative (-) input (black wire) of the boost converter to the negative terminal of the male DC Power Jack Plug Adapters.

    Cut 2 50 x 50mm square piece of Velcro Strips and attach 1 piece to the bottom of the DC 12V to 24V Boost Converter. The Wired converted will resemble Figure 19.   
    Note: The Fan Splitter Cable used has 3 4-pin female connectors, but only 2 will be used.
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 18 DC 12V to 24V Boost Converter.png" alt="">
  </div>
  <p>Figure 18 DC 12V to 24V Boost Converter</p>
</div>
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 19 Boost Converter Wired.png" alt="">
  </div>
  <p>Figure 19 Boost Converter Wired</p>
</div>

### Step 19
19.	Components needed:
    -	Fog Screen Housing (Subassembly).
    -	Fog machine

    Install the fog screen housing on top of the fog machine case aligned such it faces away from the robot. Connect the fog machine pipe into the open hole on top of the fog screen. This will resemble Figure 20. 
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 20 Projector Hose Install.png" alt="">
  </div>
  <p>Figure 20 Projector Hose Install</p>
</div>

### Step 20
20.	Components needed:
    -	Home Theater Projector charger. 
    -	Fog machine - 400W. 
    -	DC 12V to 24V Boost Converter, 3A 72W.
    -	Fog machine controller case.
    -	Industrial Strength Velcro Strips.

    Plug the projector charger into the upper port of the power station and connect the other end of the charger to the projector. Insert fog machine wire into the lower port of the power station to resemble Figure 21.  
    
    Attach the extra Velcro strip from step 18 to the front of the power station on F5. Install the boost converter onto the Velcro, ensuring firm attachment. Connect the male jack plug of the boost converter to the power station. Then, connect the 2 female fan splitter cables to the 2 male fan splitter cables of the fog screen housing, to resemble Figure 21, splitter cables are shown to the right being held.  
    
    Attach the extra Velcro strip from step 17 next to the boost converter on F3. Place the fog machine controller case onto the Velcro. Connect the USB cable of the fog machine controller to the Fetch Mobile Manipulator and plug the male XLR DMX Cable into the fog machine.  
    
    Note: If the wire length is insufficient, use wire connectors and purchase additional wire to extend.
<div style="text-align: center;">
  <div class="image">
    <img src="assembly image/Figure 21 System Wiring.png" alt="">
  </div>
  <p>Figure 21 System Wiring</p>
</div>

### Step 21
21.	Components needed:
    -	DisplayPort to HDMI Cable.

    Connect the DisplayPort to the Fetch Mobile Manipulator and the HDMI Cable to the projector.