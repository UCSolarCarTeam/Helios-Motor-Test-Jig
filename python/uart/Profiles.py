import json
from typing import List, Dict, Any

class Profiles:
    def __init__(self, file_path: str):
        self.file_path = file_path
        self.data = self._load_profiles()

    def _load_profiles(self) -> Dict[str, Any]:
        """Loads the profiles from the JSON file."""
        try:
            with open(self.file_path, 'r') as file:
                return json.load(file)
        
        except FileNotFoundError:
            raise FileNotFoundError(f"The file '{self.file_path}' does not exist.")
        
        except json.JSONDecodeError:
            raise ValueError(f"The file '{self.file_path}' contains invalid JSON.")

    def get_profiles(self) -> List[Dict[str, Any]]:
        """Returns the list of profiles."""
        return self.data.get("profiles", [])

    def get_profile_by_name(self, name: str) -> Dict[str, Any]:
        """Returns a profile by its name."""
        for profile in self.get_profiles():
            if profile.get("name") == name:
                return profile
            
        print(f"Profile with name '{name}' not found.")

    def get_commands_by_profile_name(self, name: str) -> List[str]:
        """Returns the command sequence of a profile by its name."""
        profile = self.get_profile_by_name(name)

        return profile.get("command_sequence", [])

    def add_profile(self, name: str, command_sequence: List[str]):
        """Adds a new profile."""
        if any(profile.get("name") == name for profile in self.get_profiles()):
            raise ValueError(f"Profile with name '{name}' already exists.")
        new_profile = {
            "name": name,
            "command_sequence": command_sequence
        }
        self.data["profiles"].append(new_profile)
        self._save_profiles()

    def delete_profile(self, name: str):
        """Deletes a profile by its name."""
        profiles = self.get_profiles()
        updated_profiles = [profile for profile in profiles if profile.get("name") != name]
        if len(profiles) == len(updated_profiles):
            raise ValueError(f"Profile with name '{name}' not found.")
        self.data["profiles"] = updated_profiles
        self._save_profiles()

    def _save_profiles(self):
        """Saves the current profiles to the JSON file."""
        with open(self.file_path, 'w') as file:
            json.dump(self.data, file, indent=4)