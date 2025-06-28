import sys
import xacro

def main():
    if len(sys.argv) != 3:
        print("Usage: python generate_urdf.py input.xacro output.urdf")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    # Process Xacro
    doc = xacro.process_file(input_file)
    urdf_xml = doc.toprettyxml(indent='  ')

    # Write URDF to file
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(urdf_xml)

    print(f"Generated URDF: {output_file}")

if __name__ == "__main__":
    main()
    
    
    
# python C:\Users\spes0\OneDrive\Desktop\generate_urdf.py C:\Users\spes0\OneDrive\Desktop\scarab_matlab\scarabarm_description\full_scrab_arm\urdf\full_scrab_arm.xacro C:\Users\spes0\OneDrive\Desktop\scarab_matlab\full_scrab_arm.urdf

