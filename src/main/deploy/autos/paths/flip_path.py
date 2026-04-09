import os

print("")

cmds = ["flip", "auto", "ls", "info", "cls", "re", "del", "exit"]

while True:

    cancel = False
    valid = False

    while not valid:
            cmd = input("[flip/auto/ls/info/cls/re/del/exit]: ").lower()
            if cmd in cmds:
                valid = True
            else:
                print("That command does not exist!\n")

    if cmd == "exit":
        break

    elif cmd == "del":
        valid = False

        while not valid:
            try:
                file = input("File name of path to delete: ")
                if file.lower() == "cancel":
                    cancel = True
                    break
                jsons = file.split(" ")
                new_jsons = []
                for item in jsons:
                    if not item == "*" and not ".json" in item:
                        new_jsons.append(item + ".json")
                    else:
                        new_jsons.append(item)
                jsons = new_jsons
                if not file == "*":
                    for item in jsons:  
                        with open(item, 'r') as path:
                            valid = True
                else:
                    print("There is no delete all for security.\n")                   
            except Exception:
                print("That file does not exist!\n")

        if not cancel:
            for item in jsons:
                os.remove(item)
                print(f'Deleted file: "{item}"')

    elif cmd == "re":
        valid = False

        while not valid:
            try:
                file = input("File name of path to rename: ")
                if file.lower() == "cancel":
                    cancel = True
                    break
                if ".json" not in file:
                    file += ".json"
                if "*" in file:
                    print("Cannot rename all or multiple files. (No '*' or ' '.\n")
                else:
                    with open(file, 'r') as path:
                        valid = True                
            except Exception:
                print("That file does not exist!\n")
                
        valid = False

        while not valid:
            if cancel: break
            new_file = input("New name of path: ")
            if "\\" in new_file or"/" in new_file or ":" in new_file or "*" in new_file or "?" in new_file or '"' in new_file or "<" in new_file or ">" in new_file or "|" in new_file or new_file.lower() == "cancel":
                print('These symbols cannot be in the filename: \\ / : * ? " < > | ')
                print("'cancel' and ' ' cannot be in the filename either.\n")
            else:
                valid = True

        if not cancel:
            if not ".json" in new_file:
                new_file += ".json"

            os.rename(file, new_file)
            print(f'File renamed: "{file}" -> "{new_file}"')

    elif cmd == "cls":
        if os.name == "nt":
            os.system("cls")
        else:
            os.system("clear")

    elif cmd == "info":
        print("--- BLine Auto Path Flipper ---\n")
        print("File Naming:")
        print("    Name json files like this for compatibility with flip_path.py: file_name_{blue/red}_{a/b}")
        print("    blue/red are horizontal flips (correlates to their respective sides) and a/b are vertical flips (a is top half, b is bottom half)")
        print('    These symbols cannot be in the filename: \\ / : * ? " < > | ')
        print("    'cancel' and ' ' cannot be in the filename either.\n")
        print("Syntax:")
        print("    When listing jsons (auto paths) use a space if you want to specify multiple files.")
        print("    Use '*' to specify all jsons.")
        print("    Type 'cancel' to back out of an operation.\n")
        print("Commands:")
        print("    flip - Flips the inputted json(s) as specified. v - flips vertically; h - flips horizontally; d - flips horizontally and vertically together.")
        print("    auto - Flips the inputted json(s) automatically in every possible way.")
        print("    ls - Lists all  jsons in the current working directory.")
        print("    info - Shows list of commands, instructions, and credits... but you know that.")
        print("    cls - Clears the screen")
        print("    re - Renames the inputted json. Only works with one at a time.")
        print("    del - Deletes the inputted json(s). Cannot use '*' to avoid deleting all by acident.")
        print("    exit - Closes the program.")
        print("Credits:")
        print("    A program by CatsCanCoder.")
        print("    Help Obtained From:")
        print("         - https://www.geeksforgeeks.org/python/how-to-replace-values-in-a-list-in-python/")
        print("         - https://www.geeksforgeeks.org/python/python-os-rename-method/")
        print("         - https://www.geeksforgeeks.org/python/python-break-statement/")
        print("         - https://www.geeksforgeeks.org/python/clear-screen-python/")
            
    elif cmd == "ls":
        print("Jsons:")
        files = os.listdir(os.getcwd())

        for file in files:
            if ".json" in file:
                print(file)
                
    else:

        valid = False

        while not valid:
            try:
                file = input("File name of path (Use '*' for all): ")
                if file.lower() == "cancel":
                    cancel = True
                    break
                jsons = file.split(" ")
                new_jsons = []
                for item in jsons:
                    if not item == "*" and not ".json" in item:
                        new_jsons.append(item + ".json")
                    else:
                        new_jsons.append(item)
                jsons = new_jsons
                if not file == "*":
                    for item in jsons:  
                        with open(item, 'r') as path:
                            valid = True
                else:
                    valid = True                
            except Exception:
                print("That file does not exist!\n")

        if not cancel:

            if file == "*":
                files = os.listdir(os.getcwd())
                jsons = []

                for item in files:
                    if ".json" in item:
                        jsons.append(item)

            valid = False

            if cmd == "flip":
                while not valid:
                    mode = input("Flip vertically, horizontally, or diagonal? (v/h/d): ")
                    if mode == 'v' or mode == 'h' or mode == 'd':
                        valid = True
                    else:
                        print("That mode does not exist!\n")

                    
            index = 0
            for file in jsons:
                for index in range(3):

                    if cmd == "auto":
                        if index == 0:
                            mode = 'v'
                        elif index == 1:
                            mode = 'h'
                        elif index == 2:
                            mode = 'v'
                            file_add = ""
                            if "-a" in file:
                                file_add = "-a"
                            elif "-b" in file:
                                file_add = "-b"
                            if "_blue" in file:
                                file = file.split("_blue")[0] + "_red" + file_add + ".json"
                            elif "_red" in file:
                                file = file.split("_red")[0] + "_red" + file_add + ".json"
                            else:
                                file = file.split(".json")[0] + " (3)"  + ".json"
                                
                    index_flip_check = index == 1 and cmd == "flip"
                    if index_flip_check or cmd == "auto":
                        with open(file, 'r') as path:
                            path_r_split = path.read().split(" ")
                            path_r_split_temp = []
                            raw_data = []
                            for item in path_r_split:
                                path_r_split_temp.append(item.split('\n')[0])
                            for item in  path_r_split_temp:
                                raw_data.append(item.split(',')[0])

                            if mode == 'v' or mode == 'd':
                                for i, item in enumerate(raw_data):
                                    item_add = ""
                                    if i < len(raw_data) - 1:
                                        if not path_r_split[i+1] == '}\n':
                                             item_add = ',\n'
                                        else:
                                            item_add = '\n'
                                    
                                    if item == '"y_meters":':
                                        path_r_split[i+1] = str( 8.1858 - float(raw_data[i+1]) ) + item_add

                                    # deg_change = deg - 180; deg = 180 - deg_change (3.141593rad = 180deg)
                                    if item == '"rotation_radians":':
                                        path_r_split[i+1] = str( 3.141593 - (float(raw_data[i+1]) - 3.141593) ) + item_add

                                if not mode == 'd':
                                    new_file = file.split(".json")[0].split(f"_({index+2-1})")[0] + f"_({index+2})"  + ".json"
                                    if "-a" in file:
                                        new_file = file.split("-a")[0]  + "-b" + ".json"
                                    elif "-b" in file:
                                        new_file = file.split("-b")[0]  + "-a" + ".json"

                                                
                            if mode == 'h' or mode == 'd':
                                for i, item in enumerate(raw_data):
                                    item_add = ""
                                    if i < len(raw_data)-1:
                                        if not path_r_split[i+1] == '}\n':
                                            item_add = ',\n'
                                        else:
                                            item_add = '\n'
                                    
                                    if item == '"x_meters":':
                                        path_r_split[i+1] = str( 16.7193 - float(raw_data[i+1]) ) + item_add

                                # deg = deg - 180 (3.141593rad = 180deg)
                                    if item == '"rotation_radians":':
                                        path_r_split[i+1] = str( 3.141593 - float(raw_data[i+1]) ) + item_add
                                        
                                if not mode == 'd':
                                    new_file = file.split(".json")[0].split(f"_({index+2-1})"  + ".json")[0] + f"_({index+2})"  + ".json"
                                    file_add = ""
                                    if "-a" in file:
                                        file_add = "-a"
                                    elif "-b" in file:
                                        file_add = "-b"
                                    if "_blue" in file:
                                        new_file = file.split("_blue")[0] + "_red" + file_add + ".json"
                                    elif "_red" in file:
                                        new_file = file.split("_red")[0] + "_red" + file_add + ".json"

                            if mode == 'd':
                                new_file = file.split(".json")[0].split(f"_({index+2-1})"  + ".json")[0] + f"_({index+2})"  + ".json"
                                file_add = ""
                                if "-a" in file:
                                    file_add = "-b"
                                elif "-b" in file:
                                    file_add = "-a"
                                if "_blue" in file:
                                    new_file = file.split("_blue")[0] + "_red" + file_add + ".json"
                                elif "_red" in file:
                                    new_file = file.split("_red")[0] + "_red" + file_add + ".json"


                            with open(new_file, 'w') as path:
                                file_contents = ""
                                for item in path_r_split:
                                    file_contents += item + ' '
                                    
                                path.write(file_contents)
                                print(f'File made: "{new_file}"')
                
    print("")

            
