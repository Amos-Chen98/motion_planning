#!/bin/bash

# Script to verify log file integrity after batch testing
# This script checks if the number of JSON log files matches the expected number of tests

# Get the log directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
LOG_DIR="$SCRIPT_DIR/../logs"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "============================================"
echo "Log File Integrity Verification"
echo "============================================"
echo ""

# Check if log directory exists
if [ ! -d "$LOG_DIR" ]; then
    echo -e "${RED}Error: Log directory not found: $LOG_DIR${NC}"
    exit 1
fi

# Count JSON log files
JSON_COUNT=$(find "$LOG_DIR" -name "planning_session_*.json" -type f | wc -l)

echo "Log Directory: $LOG_DIR"
echo "Total JSON log files found: $JSON_COUNT"
echo ""

# Check for most recent batch test summary
BASH_LOGS_DIR="$SCRIPT_DIR/bash_logs"
if [ -d "$BASH_LOGS_DIR" ]; then
    LATEST_SUMMARY=$(ls -t "$BASH_LOGS_DIR"/batch_test_summary_*.txt 2>/dev/null | head -1)
    
    if [ -n "$LATEST_SUMMARY" ]; then
        echo "Latest batch test summary: $(basename "$LATEST_SUMMARY")"
        
        # Extract expected test count from summary
        EXPECTED_TESTS=$(grep "Total Tests:" "$LATEST_SUMMARY" | grep -oP '\d+')
        
        if [ -n "$EXPECTED_TESTS" ]; then
            echo "Expected number of tests: $EXPECTED_TESTS"
            echo ""
            
            # Compare counts
            if [ "$JSON_COUNT" -eq "$EXPECTED_TESTS" ]; then
                echo -e "${GREEN}✓ SUCCESS: All test logs are present!${NC}"
                echo -e "${GREEN}  JSON files ($JSON_COUNT) = Expected tests ($EXPECTED_TESTS)${NC}"
                exit 0
            elif [ "$JSON_COUNT" -gt "$EXPECTED_TESTS" ]; then
                echo -e "${YELLOW}⚠ WARNING: More log files than expected${NC}"
                echo -e "${YELLOW}  JSON files ($JSON_COUNT) > Expected tests ($EXPECTED_TESTS)${NC}"
                echo -e "${YELLOW}  This may indicate logs from previous runs.${NC}"
            else
                MISSING=$((EXPECTED_TESTS - JSON_COUNT))
                echo -e "${RED}✗ FAILURE: Missing log files!${NC}"
                echo -e "${RED}  JSON files ($JSON_COUNT) < Expected tests ($EXPECTED_TESTS)${NC}"
                echo -e "${RED}  Missing $MISSING log file(s)${NC}"
                
                # List recent test timestamps from summary
                echo ""
                echo "Recent test execution times from summary:"
                grep "Started:" "$LATEST_SUMMARY" | tail -5
                
                exit 1
            fi
        fi
    fi
fi

echo ""
echo "Recent log files (last 10):"
find "$LOG_DIR" -name "planning_session_*.json" -type f -printf '%T@ %p\n' | sort -rn | head -10 | while read timestamp file; do
    date_str=$(date -d "@${timestamp%.*}" '+%Y-%m-%d %H:%M:%S')
    echo "  $date_str - $(basename "$file")"
done

echo ""
echo "============================================"
