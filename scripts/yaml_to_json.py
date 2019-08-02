import os
import yaml
import json
import argparse

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--yaml_dir")
    args = parser.parse_args()

    for filename in os.listdir(args.yaml_dir):
        if filename.endswith(".yaml"):
            dir_fn = args.yaml_dir + filename
            print(dir_fn)
            with open(dir_fn, 'r') as fin:
                config = yaml.load(fin)

            output = dir_fn.strip(".yaml")+".json"
            with open(output, 'w') as fout:
                json.dump(config, fout)
