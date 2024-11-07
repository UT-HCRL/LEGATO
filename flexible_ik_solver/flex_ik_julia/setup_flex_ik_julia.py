from pathlib import Path
from juliacall import Pkg as jlPkg

# Get the current file path
current_file_path = Path(__file__).resolve()

# Get the directory containing the current file
current_file_directory = current_file_path.parent

jlPkg.develop(path=f"{current_file_directory}/FlexIK")
