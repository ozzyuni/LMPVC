#!/usr/bin/env python3
import copy
import pickle
import time
import argparse
from pathlib import Path

class CacheEntry:
    """Stores data related to a single item in the cache"""
    def __init__(self, code: str):
        self.code = code
        self.timestamp = time.time()
        self.last_called = self.timestamp
        self.calls = 0
    
    def update(self):
        """Updates the data as the cache operates.
            This should be called whenever the data
            is served outside the cache.
        """
        self.last_called = time.time()
        self.calls += 1

        
class CodeGenCache:
    """Caches results from codegen with generated metadata.
        Persistent if given a filename.
    """

    @staticmethod
    def load_from_file(filename: str):
        """Loads file 'filename' from disk and unpickles the contents
            - Creates the file if it does not exist
            - Returns an empty dict if there's no data to unpickle
        """
        data = {}
        filepath = Path(__file__).with_name(filename)

        if not filepath.exists():
            with open(filepath, 'wb'):
                pass

        with open(filepath, 'rb') as file:
            try:
                data = pickle.load(file)
            except EOFError:
                data = {}
        
        return data

    @staticmethod
    def write_to_file(data: dict, filename: str):
        """Writes a dict to a binary file"""
        filepath = Path(__file__).with_name(filename)
        with open(filepath, 'wb') as file:
            pickle.dump(data, file)

    def __init__(self, filename=None):
        """Automatically loads data from file
            if a filename is specified
        """
        self.filename = filename
        self.data = {}
        self.checkpoint = {}
        self.load()
    
    def save(self):
        """Saves internal data to disk, if a filename was
            specified
        """
        self.checkpoint = copy.deepcopy(self.data)

        if self.filename is not None:
            self.write_to_file(self.data, self.filename)
        else:
            print("Filename not defined, saving to temporary checkpoint!")
    
    def load(self):
        """Loads internal data from disk, if a filename was
            specified
        """
        self.data = copy.deepcopy(self.checkpoint)

        if self.filename is not None:
            self.data = self.load_from_file(self.filename)
            self.checkpoint = self.data
        else:
            self.data = {}
            print("Filename not defined, loading from temporary checkpoint!")
    
    def backup(self, filename):
        """Saves internal data to disk, to specified
            backup file
        """
        self.write_to_file(self.data, filename)
    
    def add(self, prompt: str, code: str, write_to_disk=False):
        """Adds a cache entry for a certain prompt-code pair
            - Optionally writes the changes to disk
        """
        entry = CacheEntry(code)
        self.data[prompt] = entry
        
        if write_to_disk:
            self.save()
    
    def delete(self, prompt, write_to_disk=False):
        """Removes a cache entry for a given prompt
            - Optionally writes the changes to disk
        """
        if self.data.get(prompt) is not None:
            del self.data[prompt]

        if write_to_disk:
            self.save()
    
    def clear(self, write_to_disk=False):
        """Deletes the entire contents of the cache
            - Optionally writes the changes to disk
        """
        self.data = {}
        
        if write_to_disk:
            self.save()
    
    def search(self, prompt: str):
        """Search the cache for matches to a given prompt.
            If found, returns the corresponding code.
        """
        entry = self.data.get(prompt)
        code = None
        
        if entry is not None:
            code = entry.code
            entry.update()

            # Simulate the time it takes for the llm to run by waiting 0.5 seconds for every 40 characters,
            # or a maximum of 5 seconds
            wait_time = max( (len(code) / 40) * 0.5, 5 )
            time.wait(wait_time)
        
        return code

def main():
    parser = argparse.ArgumentParser()
    pgroup = parser.add_mutually_exclusive_group()

    pgroup.add_argument('-b', '--backup', help="backup source to target", action='store_true')
    pgroup.add_argument('-c', '--clear', help="clear cache from source", action='store_true')
    pgroup.add_argument('-d', '--delete', help="delete entry from source", action='store_true')
    parser.add_argument('-p', '--prompt', help="prompt to delete")
    parser.add_argument('-s', '--source', help="source file")
    parser.add_argument('-t', '--target', help="target file")

    args = parser.parse_args()

    source_found = False

    if args.backup or args.clear or args.delete:
    
        if args.source is not None:
            if Path(__file__).with_name(args.source).exists():
                source_found = True
            else:
                print("Source file not found!")
        else:
            print("Source file not specified!")
    
    else:
        print("Run 'cache -h' to see usage!")
    
    if source_found:
        cache = CodeGenCache(args.source)
        
        if args.backup:
            if args.target is not None:
                if Path(__file__).with_name(args.target).exists():
                    cache.backup(args.target)
                else:
                    print("Target file not found!")
            else:
                print("Target file not specified!")
            
        elif args.clear:
            cache.clear(write_to_disk=True)
        elif args.delete:
            cache.delete(args.prompt, write_to_disk=True)
        
if __name__ == '__main__':
    main()