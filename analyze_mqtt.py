import pyshark
import re
import json
import os
import glob
from datetime import datetime
import statistics

def clean_at_data(raw_data):
    """Clean AT command data by removing ANSI codes and extracting the actual command"""
    if not raw_data:
        return None
    
    # Remove ANSI escape sequences (color codes)
    ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
    cleaned = ansi_escape.sub('', raw_data)
    
    # Look for AT Stream content specifically
    at_stream_match = re.search(r'AT Stream:\s*([^\n]+)', cleaned)
    if at_stream_match:
        stream_content = at_stream_match.group(1).strip()
        stream_content = stream_content.replace('\\r\\n', '').strip()
        return stream_content
    
    # Look for specific AT command patterns
    at_patterns = [
        r'AT[+%][A-Z0-9]+[?:]?[^\r\n\\]*',
        r'\+[A-Z0-9]+:\s*[0-9,\s]*',
        r'AT[A-Z0-9]*[?]?'
    ]
    
    for pattern in at_patterns:
        matches = re.findall(pattern, cleaned, re.IGNORECASE)
        if matches:
            result = matches[0].strip()
            if 'Let\'s Encrypt' in result or 'lencr.org' in result or len(result) > 50:
                continue
            return result
    
    return None

def split_sessions_by_systemmode(file_path):
    """Split packets into sessions based on AT%XSYSTEMMODE commands"""
    print(f"Analyzing PCAP file: {file_path}")
    
    try:
        capture = pyshark.FileCapture(file_path)
        all_packets = []
        systemmode_commands = []
        
        # Collect all packets and find XSYSTEMMODE commands
        packet_count = 0
        for pkt in capture:
            packet_count += 1
            all_packets.append((packet_count, pkt))
            
            if hasattr(pkt, 'at'):
                raw_at_data = str(pkt.at)
                cleaned_data = clean_at_data(raw_at_data)
                
                if cleaned_data and 'AT%XSYSTEMMODE' in cleaned_data:
                    systemmode_commands.append({
                        'packet_num': packet_count,
                        'command': cleaned_data,
                        'time': pkt.sniff_time
                    })
        
        capture.close()
        
        # Find session boundaries
        ltem_start = None
        nbiot_start = None
        
        for cmd in systemmode_commands:
            command = cmd['command']
            if 'AT%XSYSTEMMODE=1,0,0' in command:
                if ltem_start is None:
                    ltem_start = cmd['packet_num']
            elif 'AT%XSYSTEMMODE=0,1,0' in command:
                nbiot_start = cmd['packet_num']
                break
        
        # Split packets into sessions
        sessions = {}
        
        if ltem_start and nbiot_start:
            ltem_packets = all_packets[ltem_start-1:nbiot_start-1]
            sessions['LTE-M'] = ltem_packets
            
            nbiot_packets = all_packets[nbiot_start-1:]
            sessions['NB-IoT'] = nbiot_packets
        
        elif ltem_start:
            ltem_packets = all_packets[ltem_start-1:]
            sessions['LTE-M'] = ltem_packets
        
        return sessions
        
    except Exception as e:
        print(f"Error reading file {file_path}: {e}")
        return None

def calculate_latency(packets):
    """Calculate average latency from TCP ACK delays"""
    latencies = []
    tcp_packets = {}
    
    for original_packet_num, pkt in packets:
        if hasattr(pkt, 'tcp'):
            if hasattr(pkt.tcp, 'dstport') and int(pkt.tcp.dstport) == 8883:
                if hasattr(pkt.tcp, 'seq'):
                    seq = int(pkt.tcp.seq)
                    tcp_packets[seq] = pkt.sniff_time
            
            elif hasattr(pkt.tcp, 'srcport') and int(pkt.tcp.srcport) == 8883:
                if hasattr(pkt.tcp, 'ack') and hasattr(pkt.tcp, 'flags'):
                    flags = int(pkt.tcp.flags, 16)
                    if flags & 0x10:  # ACK flag
                        ack = int(pkt.tcp.ack)
                        if ack in tcp_packets:
                            latency = (pkt.sniff_time - tcp_packets[ack]).total_seconds() * 1000
                            if 0 < latency < 5000:  # Filter reasonable latencies
                                latencies.append(latency)
    
    if latencies:
        return statistics.mean(latencies)
    return None

def calculate_retransmissions(packets):
    """Calculate TCP retransmissions for MQTT traffic"""
    retransmissions = 0
    tcp_packets = {}
    
    for original_packet_num, pkt in packets:
        if hasattr(pkt, 'tcp'):
            # Look for MQTT port (8883)
            if hasattr(pkt.tcp, 'dstport') and int(pkt.tcp.dstport) == 8883:
                if hasattr(pkt.tcp, 'seq') and hasattr(pkt.tcp, 'len'):
                    seq = int(pkt.tcp.seq)
                    length = int(pkt.tcp.len)
                    
                    # Check for retransmission (same seq number seen before)
                    packet_signature = f"{seq}:{length}"
                    if packet_signature in tcp_packets:
                        retransmissions += 1
                    else:
                        tcp_packets[packet_signature] = pkt.sniff_time
    
    return retransmissions

def calculate_tls_handshake_time(packets):
    """Calculate TLS handshake time"""
    tls_start_time = None
    tls_end_time = None
    
    for original_packet_num, pkt in packets:
        if hasattr(pkt, 'tcp'):
            # Look for MQTT port (8883) with data
            if hasattr(pkt.tcp, 'dstport') and int(pkt.tcp.dstport) == 8883:
                if hasattr(pkt.tcp, 'payload') and len(str(pkt.tcp.payload)) > 20:
                    if tls_start_time is None:
                        tls_start_time = pkt.sniff_time
                    else:
                        # Look for larger payload indicating handshake completion
                        if len(str(pkt.tcp.payload)) > 100:
                            tls_end_time = pkt.sniff_time
    
    if tls_start_time and tls_end_time:
        return (tls_end_time - tls_start_time).total_seconds() * 1000  # Return in ms
    return None

def calculate_rrc_state_changes(packets):
    """Count RRC state changes"""
    rrc_state_changes = 0
    
    for original_packet_num, pkt in packets:
        if hasattr(pkt, 'at'):
            raw_at_data = str(pkt.at)
            cleaned_data = clean_at_data(raw_at_data)
            
            if cleaned_data and '+CSCON:' in cleaned_data:
                rrc_state_changes += 1
    
    return rrc_state_changes

def calculate_network_registration_changes(packets):
    """Count network registration changes"""
    network_registrations = []
    
    for original_packet_num, pkt in packets:
        if hasattr(pkt, 'at'):
            raw_at_data = str(pkt.at)
            cleaned_data = clean_at_data(raw_at_data)
            
            if cleaned_data and '+CEREG:' in cleaned_data:
                match = re.search(r'\+CEREG:\s*(\d+)', cleaned_data)
                if match:
                    reg_status = int(match.group(1))
                    network_registrations.append({
                        'time': pkt.sniff_time,
                        'status': reg_status
                    })
    
    return len(network_registrations)

def analyze_mqtt_performance(packets, session_name):
    """Analyze MQTT performance for a session - only 5 metrics"""
    print(f"  Analyzing {session_name} performance...")
    
    # Calculate only the 5 specified metrics
    metrics = {
        'latency': calculate_latency(packets),
        'retransmissions': calculate_retransmissions(packets),
        'tls_handshake_time': calculate_tls_handshake_time(packets),
        'rrc_state_changes': calculate_rrc_state_changes(packets),
        'network_registration_changes': calculate_network_registration_changes(packets)
    }
    
    return metrics

def process_measurement(measurement_path):
    """Process a single measurement folder"""
    pcap_file = os.path.join(measurement_path, 'mqtt_trace.pcapng')
    json_file = os.path.join(measurement_path, 'all_data.json')
    
    if not os.path.exists(pcap_file):
        print(f"PCAP file not found: {pcap_file}")
        return False
    
    if not os.path.exists(json_file):
        print(f"JSON file not found: {json_file}")
        return False
    
    print(f"\nProcessing: {measurement_path}")
    
    # Split sessions
    sessions = split_sessions_by_systemmode(pcap_file)
    
    if not sessions:
        print(f"Could not split sessions for {measurement_path}")
        return False
    
    # Load existing JSON data
    try:
        with open(json_file, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print(f"Error loading JSON: {e}")
        return False
    
    # Analyze each session and add metrics to JSON
    for session_name, packets in sessions.items():
        if session_name == 'LTE-M':
            json_key = 'mqtt_data_ltem'
        elif session_name == 'NB-IoT':
            json_key = 'mqtt_data_nbiot'
        else:
            continue
        
        if json_key not in data:
            print(f"JSON key {json_key} not found in {json_file}")
            continue
        
        # Calculate only the 5 specified performance metrics
        metrics = analyze_mqtt_performance(packets, session_name)
        
        # Add metrics to JSON data
        data[json_key].update(metrics)
        
        print(f"    {session_name} metrics calculated:")
        for key, value in metrics.items():
            if value is not None:
                if isinstance(value, float):
                    print(f"      {key}: {value:.2f}")
                else:
                    print(f"      {key}: {value}")
            else:
                print(f"      {key}: N/A")
    
    # Save updated JSON
    try:
        with open(json_file, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"  Updated: {json_file}")
        return True
    except Exception as e:
        print(f"Error saving JSON: {e}")
        return False

def main():
    """Main function to process all measurements"""
    measurements_dir = 'Measurements'
    
    if not os.path.exists(measurements_dir):
        print(f"Measurements directory not found: {measurements_dir}")
        return
    
    # Find all measurement folders
    measurement_folders = glob.glob(os.path.join(measurements_dir, 'measurement_*'))
    measurement_folders.sort()
    
    print(f"Found {len(measurement_folders)} measurement folders")
    
    successful = 0
    failed = 0
    
    for folder in measurement_folders:
        try:
            if process_measurement(folder):
                successful += 1
            else:
                failed += 1
        except Exception as e:
            print(f"Error processing {folder}: {e}")
            failed += 1
    
    print(f"\n{'='*60}")
    print("PROCESSING SUMMARY")
    print(f"{'='*60}")
    print(f"Total measurements: {len(measurement_folders)}")
    print(f"Successful: {successful}")
    print(f"Failed: {failed}")
    print(f"Success rate: {(successful/len(measurement_folders)*100):.1f}%")

if __name__ == "__main__":
    main()
