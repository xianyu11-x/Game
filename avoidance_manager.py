"""
Avoidance plugin manager
Loads and manages avoidance algorithms from plugin files
"""

import os
import sys
import importlib.util

class AvoidanceManager:
    """Manages loading and switching between avoidance algorithms"""
    
    def __init__(self):
        self.algorithms = {}
        self.current_algorithm = None
        self.plugin_dir = os.path.dirname(os.path.abspath(__file__))
    
    def load_plugins(self):
        """Load all avoidance plugins from the plugin directory"""
        # Get all Python files starting with 'avoidance_' (except base)
        plugin_files = [f for f in os.listdir(self.plugin_dir) 
                       if f.startswith('avoidance_') and f.endswith('.py') 
                       and f != 'avoidance_base.py' and f != 'avoidance_manager.py']
        
        for plugin_file in plugin_files:
            self._load_plugin(plugin_file)
        
        # Set default algorithm if available
        if self.algorithms and not self.current_algorithm:
            self.current_algorithm = list(self.algorithms.values())[0]
        
        return len(self.algorithms)
    
    def _load_plugin(self, filename):
        """Load a single plugin file"""
        try:
            module_name = filename[:-3]  # Remove .py extension
            file_path = os.path.join(self.plugin_dir, filename)
            
            # Load the module
            spec = importlib.util.spec_from_file_location(module_name, file_path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            
            # Find all classes that inherit from AvoidanceBase
            from avoidance_base import AvoidanceBase
            for attr_name in dir(module):
                attr = getattr(module, attr_name)
                if (isinstance(attr, type) and 
                    issubclass(attr, AvoidanceBase) and 
                    attr is not AvoidanceBase):
                    # Instantiate the algorithm
                    algorithm = attr()
                    self.algorithms[algorithm.get_name()] = algorithm
                    print(f"Loaded avoidance plugin: {algorithm.get_name()} - {algorithm.get_description()}")
        
        except Exception as e:
            print(f"Error loading plugin {filename}: {e}")
    
    def get_algorithm(self, name=None):
        """Get an algorithm by name, or return current algorithm"""
        if name is None:
            return self.current_algorithm
        return self.algorithms.get(name)
    
    def set_algorithm(self, name):
        """Set the current algorithm by name"""
        if name in self.algorithms:
            self.current_algorithm = self.algorithms[name]
            return True
        return False
    
    def get_algorithm_names(self):
        """Get list of all available algorithm names"""
        return list(self.algorithms.keys())
    
    def next_algorithm(self):
        """Switch to the next algorithm"""
        if not self.algorithms:
            return None
        
        names = self.get_algorithm_names()
        if self.current_algorithm:
            current_name = self.current_algorithm.get_name()
            if current_name in names:
                current_index = names.index(current_name)
                next_index = (current_index + 1) % len(names)
                self.current_algorithm = self.algorithms[names[next_index]]
        else:
            self.current_algorithm = self.algorithms[names[0]]
        
        return self.current_algorithm
