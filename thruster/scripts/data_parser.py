'''
MOVE THIS SOMEWHERE ELSE
This folder should only contain things that are directly used and
related to mantaray_control.

Author: William Y.
'''

import pandas as pd
import os

def parse_data():
    SHEET_NAMES = ["10 V", "12 V", "14 V", "16 V", "18 V", "20 V"]
    DATA_DIR = os.path.join(os.getcwd(), "data", "t200_data")
    DATA_PATH = os.path.join(DATA_DIR, "t200_data.xlsx")
    columns = {"pwm": " PWM (µs)", "rpm":" RPM", "cur":" Current (A)", "volt":" Voltage (V)","pow":" Power (W)", "force":" Force (Kg f)", "eff":" Efficiency (g/W)"}

    useful_cols = [[columns["force"], columns["pwm"]],[columns["force"], columns["cur"]],[columns["force"], columns["pow"]],[columns["force"], columns["eff"]],]
    
    for sheet in SHEET_NAMES:
        current_dir = os.path.join(DATA_DIR, sheet)
        data = pd.read_excel(
            DATA_PATH, sheet_name=sheet,
            engine='openpyxl',
        )
        for cols in useful_cols:
            df = data.filter(items=cols)
            key0 = list(filter(lambda x: columns[x] == cols[0], columns))[0]
            key1 = list(filter(lambda x: columns[x] == cols[1], columns))[0]
            
            file_path = os.path.join(current_dir, f"{key0}_{key1}.csv")

            df.to_csv(file_path)
            # print(f"This would be in a file called {file_name}.csv and at location: {current_dir}")

def modify_force(file_path="", new_file_path="", multiplier=1):
    data = pd.read_csv(file_path)
    fixed_data = data.drop(columns=['Unnamed: 0'])
    fixed_data[fixed_data.select_dtypes(include=['float64']).columns] *= multiplier
    fixed_data.to_csv(new_file_path)

def print_csv(file_path, format=""):
    # format = "xacro" prints values to match the xacro:thruster_cf_linear_interp_macro format
    data = pd.read_csv(file_path)
    data = data.drop(columns=["Unnamed: 0"])
    cols = data.columns
    if (format == "xacro"):
        print("<xacro:thruster_cf_linear_interp_macro")
    for col in cols:
        col_data = (data.filter(items=[str(col)])).values
        col_data = col_data.flatten()
        if (format == "xacro"):
            if (col==" PWM (µs)"): print("  input_values=\"", end="")
            else: print("  output_values=\"", end="")
        for val in col_data:
            print(val, end=" ")
        if (format == "xacro"): print("\"", end="")
        print("")
    print("/>")

if __name__ == "__main__":
    # Run the code below to give a multiplier to any force by PWM csv. This is for simulation use ONLY
    # DATA_DIR = os.path.join(os.getcwd(), "data", "t200_data")
    # file_path = os.path.join(DATA_DIR, "20 V", "force_pwm.csv")
    # new_file_path = os.path.join(DATA_DIR, "twice_20v_force_pwm.csv")
    # modify_force(file_path, new_file_path, multiplier=3)


    # Run the code below if you want to print out the force and pwm values of any force_pwm.csv file

    DATA_DIR = os.path.join(os.getcwd(), "data", "t200_data")
    file_path = os.path.join(DATA_DIR, "twice_20v_force_pwm.csv")
    print_csv(file_path, "xacro")
    # file_path = os.path.join(DATA_DIR, "20 V", "force_pwm.csv")
    # print_csv(file_path, 1)
    # print_csv(file_path, 2)