"""Build a sample 4-wheel robot assembly STEP to test step2drawings.py."""
import cadquery as cq

# Chassis plate with corner holes and a cable slot
chassis = (cq.Workplane("XY").box(300, 200, 5)
           .faces(">Z").workplane()
           .rect(270, 170, forConstruction=True).vertices().hole(5)
           .center(0, 0).slot2D(80, 16).cutThruAll()
           .faces(">Z").workplane()
           .pushPoints([(-110, 60), (110, 60), (-110, -60), (110, -60)]).hole(3.4))

# L-shaped motor bracket with a motor bore and mounting holes
bracket = (cq.Workplane("XY").box(50, 40, 4).translate((0, 0, 2))
           .union(cq.Workplane("XZ").box(50, 50, 4).translate((0, 18, 25 + 4 - 2 + 2)))
           .faces("<Z").workplane().pushPoints([(-15, 5), (15, 5)]).hole(3.4))
bracket = (bracket.faces(">Y").workplane(centerOption="CenterOfBoundBox")
           .hole(12)
           .faces(">Y").workplane(centerOption="CenterOfBoundBox")
           .rect(30, 30, forConstruction=True).vertices().hole(3))

# Wheel: disc with hub bore and 6 lightening holes
wheel = (cq.Workplane("XY").circle(50).extrude(25)
         .faces(">Z").workplane().hole(6)
         .faces(">Z").workplane().polarArray(30, 0, 360, 6).hole(14))

# Standoff
standoff = cq.Workplane("XY").polygon(6, 8).extrude(40).faces(">Z").workplane().hole(3)

# Top plate
top = (cq.Workplane("XY").box(280, 180, 3)
       .faces(">Z").workplane().rect(250, 150, forConstruction=True).vertices().hole(3.4)
       .faces(">Z").workplane().rect(120, 60).cutThruAll())

asm = cq.Assembly(name="robot")
asm.add(chassis, name="chassis_plate", color=cq.Color("gray"))
asm.add(top, name="top_plate", loc=cq.Location((0, 0, 45)), color=cq.Color("gray"))
for i, (x, y, s) in enumerate([(-110, 60, 1), (110, 60, 1), (-110, -60, -1), (110, -60, -1)]):
    rot = 0 if s > 0 else 180
    asm.add(bracket, name=f"motor_bracket_{i+1}",
            loc=cq.Location((x, y, 2.5), (0, 0, 1), rot), color=cq.Color("orange"))
    asm.add(wheel, name=f"wheel_{i+1}",
            loc=cq.Location((x, y + s * 75, 25), (1, 0, 0), -90 * s), color=cq.Color("black"))
for i, (x, y) in enumerate([(-125, -75), (125, -75), (-125, 75), (125, 75)]):
    asm.add(standoff, name=f"standoff_{i+1}", loc=cq.Location((x, y, 2.5)))

asm.export("sample_robot.step")
print("wrote sample_robot.step")
