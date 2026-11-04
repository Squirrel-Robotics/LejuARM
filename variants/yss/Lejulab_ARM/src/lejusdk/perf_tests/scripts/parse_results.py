#!/usr/bin/env python3
"""
CycloneDDS 性能测试结果解析器

解析 LatencyStats::print_summary() 的输出，并导出为 CSV/JSON/Table 格式。

用法:
    ./parse_results.py <log_file> [options]
    ./parse_results.py <log_dir> --batch [options]

示例:
    ./parse_results.py test_output.log --format table
    ./parse_results.py test_output.log --format csv --output results.csv
    ./parse_results.py results_dir/ --batch --format json
"""

import argparse
import json
import os
import re
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Optional, Dict, Any


@dataclass
class LatencyResult:
    """Parsed latency statistics from a single test block"""
    title: str
    topic: Optional[str] = None
    message_size: Optional[int] = None
    publish_rate: Optional[float] = None
    messages: Optional[int] = None
    min_us: Optional[float] = None
    max_us: Optional[float] = None
    mean_us: Optional[float] = None
    median_us: Optional[float] = None
    p95_us: Optional[float] = None
    p99_us: Optional[float] = None
    stddev_us: Optional[float] = None
    jitter_us: Optional[float] = None


class LatencyStatsParser:
    """Parser for LatencyStats::print_summary() output format"""

    # Regex patterns for parsing
    TITLE_PATTERN = re.compile(r'=+\s+(.+?)\s+=+')
    TOPIC_PATTERN = re.compile(r'Topic:\s*(.+)')
    MSG_SIZE_PATTERN = re.compile(r'Message Size:\s*(\d+)\s*bytes')
    RATE_PATTERN = re.compile(r'Publish Rate:\s*([\d.]+)\s*Hz')
    MESSAGES_PATTERN = re.compile(r'Messages:\s*(\d+)')

    # Latency value patterns
    LATENCY_PATTERNS = {
        'min_us': re.compile(r'Min:\s*([\d.]+)\s*us'),
        'max_us': re.compile(r'Max:\s*([\d.]+)\s*us'),
        'mean_us': re.compile(r'Mean:\s*([\d.]+)\s*us'),
        'median_us': re.compile(r'Median:\s*([\d.]+)\s*us'),
        'p95_us': re.compile(r'P95:\s*([\d.]+)\s*us'),
        'p99_us': re.compile(r'P99:\s*([\d.]+)\s*us'),
        'stddev_us': re.compile(r'Std Dev:\s*([\d.]+)\s*us'),
        'jitter_us': re.compile(r'Jitter:\s*([\d.]+)\s*us'),
    }

    def parse_file(self, filepath: str) -> List[LatencyResult]:
        """Parse a log file and extract all latency result blocks"""
        with open(filepath, 'r') as f:
            content = f.read()
        return self.parse_content(content)

    def parse_content(self, content: str) -> List[LatencyResult]:
        """Parse content string and extract all latency result blocks"""
        results = []

        # Split by title lines (lines starting with multiple '=')
        blocks = re.split(r'\n(?==+\s)', content)

        for block in blocks:
            result = self._parse_block(block)
            if result is not None:
                results.append(result)

        return results

    def _parse_block(self, block: str) -> Optional[LatencyResult]:
        """Parse a single statistics block"""
        # Check if this block contains latency statistics
        title_match = self.TITLE_PATTERN.search(block)
        if not title_match:
            return None

        # Check if block has latency data
        if 'Latency Statistics' not in block and 'us' not in block:
            return None

        title = title_match.group(1).strip()
        result = LatencyResult(title=title)

        # Parse metadata
        topic_match = self.TOPIC_PATTERN.search(block)
        if topic_match:
            result.topic = topic_match.group(1).strip()

        size_match = self.MSG_SIZE_PATTERN.search(block)
        if size_match:
            result.message_size = int(size_match.group(1))

        rate_match = self.RATE_PATTERN.search(block)
        if rate_match:
            result.publish_rate = float(rate_match.group(1))

        messages_match = self.MESSAGES_PATTERN.search(block)
        if messages_match:
            result.messages = int(messages_match.group(1))

        # Parse latency values
        for field, pattern in self.LATENCY_PATTERNS.items():
            match = pattern.search(block)
            if match:
                setattr(result, field, float(match.group(1)))

        # Only return if we found at least some latency data
        if result.min_us is not None or result.mean_us is not None:
            return result

        return None


class ResultFormatter:
    """Format parsed results to various output formats"""

    @staticmethod
    def to_csv(results: List[LatencyResult], include_header: bool = True) -> str:
        """Convert results to CSV format"""
        if not results:
            return ""

        headers = [
            'title', 'topic', 'message_size', 'publish_rate', 'messages',
            'min_us', 'max_us', 'mean_us', 'median_us', 'p95_us', 'p99_us',
            'stddev_us', 'jitter_us'
        ]

        lines = []
        if include_header:
            lines.append(','.join(headers))

        for result in results:
            row = []
            for h in headers:
                val = getattr(result, h)
                if val is None:
                    row.append('')
                elif isinstance(val, str):
                    # Escape commas and quotes in strings
                    val = val.replace('"', '""')
                    if ',' in val or '"' in val:
                        val = f'"{val}"'
                    row.append(val)
                else:
                    row.append(str(val))
            lines.append(','.join(row))

        return '\n'.join(lines)

    @staticmethod
    def to_json(results: List[LatencyResult], pretty: bool = True) -> str:
        """Convert results to JSON format"""
        data = [asdict(r) for r in results]
        if pretty:
            return json.dumps(data, indent=2)
        return json.dumps(data)

    @staticmethod
    def to_table(results: List[LatencyResult]) -> str:
        """Convert results to human-readable table format"""
        if not results:
            return "No results found."

        lines = []

        for i, result in enumerate(results):
            if i > 0:
                lines.append("")

            lines.append(f"{'=' * 60}")
            lines.append(f"  {result.title}")
            lines.append(f"{'=' * 60}")

            if result.topic:
                lines.append(f"  Topic:        {result.topic}")
            if result.message_size:
                lines.append(f"  Message Size: {result.message_size} bytes")
            if result.publish_rate:
                lines.append(f"  Publish Rate: {result.publish_rate} Hz")
            if result.messages:
                lines.append(f"  Messages:     {result.messages}")

            lines.append(f"{'-' * 60}")
            lines.append("  Latency Statistics (microseconds):")

            if result.min_us is not None:
                lines.append(f"    Min:      {result.min_us:>10.1f} us")
            if result.max_us is not None:
                lines.append(f"    Max:      {result.max_us:>10.1f} us")
            if result.mean_us is not None:
                lines.append(f"    Mean:     {result.mean_us:>10.1f} us")
            if result.median_us is not None:
                lines.append(f"    Median:   {result.median_us:>10.1f} us")
            if result.p95_us is not None:
                lines.append(f"    P95:      {result.p95_us:>10.1f} us")
            if result.p99_us is not None:
                lines.append(f"    P99:      {result.p99_us:>10.1f} us")
            if result.stddev_us is not None:
                lines.append(f"    Std Dev:  {result.stddev_us:>10.1f} us")
            if result.jitter_us is not None:
                lines.append(f"    Jitter:   {result.jitter_us:>10.1f} us")

            lines.append(f"{'=' * 60}")

        return '\n'.join(lines)

    @staticmethod
    def to_markdown_summary(results: List[LatencyResult]) -> str:
        """Convert results to Markdown table format"""
        if not results:
            return "No results found."

        lines = [
            "| Title | Topic | Size | Rate | Count | Mean (us) | P95 (us) | P99 (us) |",
            "|-------|-------|------|------|-------|-----------|----------|----------|"
        ]

        for result in results:
            topic = result.topic or "-"
            size = f"{result.message_size}" if result.message_size else "-"
            rate = f"{result.publish_rate:.0f}" if result.publish_rate else "-"
            count = f"{result.messages}" if result.messages else "-"
            mean = f"{result.mean_us:.1f}" if result.mean_us else "-"
            p95 = f"{result.p95_us:.1f}" if result.p95_us else "-"
            p99 = f"{result.p99_us:.1f}" if result.p99_us else "-"

            lines.append(f"| {result.title} | {topic} | {size} | {rate} | {count} | {mean} | {p95} | {p99} |")

        return '\n'.join(lines)


def parse_directory(dir_path: str, parser: LatencyStatsParser) -> Dict[str, List[LatencyResult]]:
    """Parse all log files in a directory"""
    results = {}
    path = Path(dir_path)

    for log_file in path.glob('*.log'):
        try:
            file_results = parser.parse_file(str(log_file))
            if file_results:
                results[log_file.name] = file_results
        except Exception as e:
            print(f"Warning: Failed to parse {log_file}: {e}", file=sys.stderr)

    # Also check for .txt files
    for log_file in path.glob('*.txt'):
        try:
            file_results = parser.parse_file(str(log_file))
            if file_results:
                results[log_file.name] = file_results
        except Exception as e:
            print(f"Warning: Failed to parse {log_file}: {e}", file=sys.stderr)

    return results


def main():
    parser = argparse.ArgumentParser(
        description='Parse CycloneDDS performance test results',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('input', help='Input log file or directory')
    parser.add_argument('-f', '--format', choices=['csv', 'json', 'table', 'markdown'],
                        default='table', help='Output format (default: table)')
    parser.add_argument('-o', '--output', help='Output file (default: stdout)')
    parser.add_argument('--batch', action='store_true',
                        help='Process all log files in directory')
    parser.add_argument('--no-header', action='store_true',
                        help='Omit CSV header (only for CSV format)')
    parser.add_argument('--compact', action='store_true',
                        help='Compact JSON output (no indentation)')

    args = parser.parse_args()

    stats_parser = LatencyStatsParser()
    formatter = ResultFormatter()

    # Parse input
    if args.batch or os.path.isdir(args.input):
        if not os.path.isdir(args.input):
            print(f"Error: {args.input} is not a directory", file=sys.stderr)
            sys.exit(1)

        all_results = parse_directory(args.input, stats_parser)

        if not all_results:
            print("No results found in directory.", file=sys.stderr)
            sys.exit(1)

        # Flatten results for output
        flat_results = []
        for filename, results in sorted(all_results.items()):
            flat_results.extend(results)

        results = flat_results
    else:
        if not os.path.isfile(args.input):
            print(f"Error: {args.input} not found", file=sys.stderr)
            sys.exit(1)

        results = stats_parser.parse_file(args.input)

        if not results:
            print("No latency statistics found in input.", file=sys.stderr)
            sys.exit(1)

    # Format output
    if args.format == 'csv':
        output = formatter.to_csv(results, include_header=not args.no_header)
    elif args.format == 'json':
        output = formatter.to_json(results, pretty=not args.compact)
    elif args.format == 'markdown':
        output = formatter.to_markdown_summary(results)
    else:  # table
        output = formatter.to_table(results)

    # Write output
    if args.output:
        with open(args.output, 'w') as f:
            f.write(output)
            f.write('\n')
        print(f"Results written to {args.output}")
    else:
        print(output)


if __name__ == '__main__':
    main()
