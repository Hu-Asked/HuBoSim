outputType = input();
if outputType == "cpp":
    with open("path.txt", "r") as f:
        for line in f:
            items = line.strip().split(",")
            print(f"{{{{{items[0]}, {items[1]}}}, {items[2]}}}")
else:
    with open("path.txt", "r") as f:
        for line in f:
            items = line.strip().split(",")
            print(f"path.add(point({items[0]}, {items[1]}, {items[2]}));")

