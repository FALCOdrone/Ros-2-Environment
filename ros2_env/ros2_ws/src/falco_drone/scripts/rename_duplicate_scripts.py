import re
import sys
from collections import defaultdict
from pathlib import Path

def rename_duplicate_links(file_path: str):
    with open(file_path, 'r') as file:
        content = file.read()

    # Match all link tags
    link_pattern = r'<link\s+name="([^"]+)"'
    links = re.findall(link_pattern, content)

    # Count duplicates
    name_counts = defaultdict(int)
    new_name_map = {}
    for name in links:
        name_counts[name] += 1
        if name_counts[name] > 1:
            new_name = f"{name}_{name_counts[name]}"
            new_name_map[(name, name_counts[name])] = new_name

    if not new_name_map:
        print("No duplicate links found. File is clean.")
        return

    # Function to rename link tags
    def replace_links(match):
        name = match.group(1)
        count = name_counts[name]
        if count > 1:
            name_counts[name] -= 1  # reverse loop fix
            suffix = name_counts[name] + 1
            new_name = f'{name}_{suffix}'
            return f'<link name="{new_name}"'
        return match.group(0)

    # Reset name_counts for replacement loop
    name_counts = defaultdict(int)
    for name in links:
        name_counts[name] += 1

    # Replace <link name="...">
    content = re.sub(r'<link\s+name="([^"]+)"', replace_links, content)

    # Replace references in <joint><parent> and <joint><child>
    for (old_name, idx), new_name in new_name_map.items():
        content = re.sub(rf'<parent\s+link="{old_name}"', f'<parent link="{new_name}"', content)
        content = re.sub(rf'<child\s+link="{old_name}"', f'<child link="{new_name}"', content)

    # Output to new file
    output_path = Path(file_path).with_name(f"{Path(file_path).stem}_renamed{Path(file_path).suffix}")
    with open(output_path, 'w') as file:
        file.write(content)

    print(f"Duplicate links renamed. Updated file written to: {output_path}")

# Usage: python rename_duplicate_links.py your_file.urdf
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python rename_duplicate_links.py <your_file.urdf|.xacro>")
    else:
        rename_duplicate_links(sys.argv[1])
