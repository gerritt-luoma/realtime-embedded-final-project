import sys
import re
from collections import defaultdict
from statistics import mean

def parse_log(file_path):
    # Matches any log with format like: SEQ CAPTURE START 123.456 or SEQ SAVE DUR 0.000740
    pattern = re.compile(r".*?\b(SEQ\s+\S+)\s+(POST|START|END|DUR)\s+([\d.]+)$")
    entries = defaultdict(list)

    with open(file_path, 'r') as f:
        for line in f:
            match = pattern.match(line.strip())
            if match:
                prefix, tag, value = match.groups()
                entries[prefix.strip()].append((tag, float(value)))

    return entries

def process_entries(entries):
    results = {}

    for prefix, events in entries.items():
        if prefix == "SEQ SAVE":
            # Special case: SEQ SAVE has only DUR lines
            durations = [val for tag, val in events if tag == "DUR"]
            if durations:
                results[prefix] = {
                    'dur_best': min(durations),
                    'dur_worst': max(durations),
                    'dur_avg': mean(durations),
                    'count': len(durations),
                    'type': 'dur_only'
                }
        else:
            i = 0
            post_start_deltas = []
            durations = []

            while i < len(events) - 3:
                tags = [events[i + j][0] for j in range(4)]
                if tags == ['POST', 'START', 'END', 'DUR']:
                    post_time = events[i][1]
                    start_time = events[i + 1][1]
                    dur_time = events[i + 3][1]

                    post_start_deltas.append(start_time - post_time)
                    durations.append(dur_time)
                    i += 4
                else:
                    i += 1

            if post_start_deltas:
                results[prefix] = {
                    'post_start_best': min(post_start_deltas),
                    'post_start_worst': max(post_start_deltas),
                    'post_start_avg': mean(post_start_deltas),
                    'dur_best': min(durations),
                    'dur_worst': max(durations),
                    'dur_avg': mean(durations),
                    'count': len(post_start_deltas),
                    'type': 'full'
                }

    return results

def print_results(results):
    for prefix, stats in results.items():
        print(f"\n=== Stats for {prefix} ({stats['count']} entries) ===")
        if stats['type'] == 'dur_only':
            print(f"DUR durations:")
            print(f"  Best : {stats['dur_best']:.9f} s")
            print(f"  Worst: {stats['dur_worst']:.9f} s")
            print(f"  Avg  : {stats['dur_avg']:.9f} s")
        else:
            print(f"Post → Start delay:")
            print(f"  Best : {stats['post_start_best']:.9f} s")
            print(f"  Worst: {stats['post_start_worst']:.9f} s")
            print(f"  Avg  : {stats['post_start_avg']:.9f} s")

            print(f"DUR durations:")
            print(f"  Best : {stats['dur_best']:.9f} s")
            print(f"  Worst: {stats['dur_worst']:.9f} s")
            print(f"  Avg  : {stats['dur_avg']:.9f} s")

def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <log_file>")
        sys.exit(1)

    file_path = sys.argv[1]
    entries = parse_log(file_path)
    results = process_entries(entries)
    print_results(results)

if __name__ == "__main__":
    main()
