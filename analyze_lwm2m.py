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

def detect_lwm2m_ports(packets):
    """Detect LwM2M communication ports"""
    lwm2m_ports = set()
    udp_traffic = {}
    
    for original_packet_num, pkt in packets:
        if hasattr(pkt, 'udp'):
            # Standard CoAP/LwM2M ports
            standard_ports = [5683, 5684, 5000, 8000, 1234, 56830]
            
            if hasattr(pkt.udp, 'dstport'):
                dst_port = int(pkt.udp.dstport)
                if dst_port in standard_ports:
                    lwm2m_ports.add(dst_port)
                else:
                    # Track unusual UDP traffic that might be LwM2M
                    if hasattr(pkt, 'ip'):
                        key = f"{pkt.ip.dst}:{dst_port}"
                        udp_traffic[key] = udp_traffic.get(key, 0) + 1
            
            if hasattr(pkt.udp, 'srcport'):
                src_port = int(pkt.udp.srcport)
                if src_port in standard_ports:
                    lwm2m_ports.add(src_port)
    
    # If no standard ports found, look for frequent UDP communication
    if not lwm2m_ports:
        if udp_traffic:
            most_frequent = max(udp_traffic.items(), key=lambda x: x[1])
            if most_frequent[1] > 2:  # At least 3 packets
                port = int(most_frequent[0].split(':')[1])
                lwm2m_ports.add(port)
    
    return list(lwm2m_ports)

def calculate_latency(packets):
    """Calculate average latency from CoAP request-response times"""
    lwm2m_ports = detect_lwm2m_ports(packets)
    latencies = []
    pending_requests = []
    
    for original_packet_num, pkt in packets:
        if hasattr(pkt, 'udp') and hasattr(pkt, 'ip'):
            # Outgoing requests (to LwM2M server)
            if hasattr(pkt.udp, 'dstport') and int(pkt.udp.dstport) in lwm2m_ports:
                pending_requests.append({
                    'time': pkt.sniff_time,
                    'dst_ip': pkt.ip.dst,
                    'dst_port': int(pkt.udp.dstport)
                })
            
            # Incoming responses (from LwM2M server)
            elif hasattr(pkt.udp, 'srcport') and int(pkt.udp.srcport) in lwm2m_ports:
                src_ip = pkt.ip.src
                src_port = int(pkt.udp.srcport)
                
                # Find matching request
                for i, req in enumerate(pending_requests):
                    if req['dst_ip'] == src_ip and req['dst_port'] == src_port:
                        latency = (pkt.sniff_time - req['time']).total_seconds() * 1000
                        if 0 < latency < 5000:  # Filter reasonable latencies
                            latencies.append(latency)
                        pending_requests.pop(i)
                        break
    
    if latencies:
        return statistics.mean(latencies)
    return None


def calculate_retransmissions(packets):
    """Improved CoAP retransmission detection"""
    lwm2m_ports = detect_lwm2m_ports(packets)
    retransmissions = 0
    coap_messages = {}
    
    for original_packet_num, pkt in packets:
        if hasattr(pkt, 'udp') and hasattr(pkt, 'ip'):
            if hasattr(pkt.udp, 'dstport') and int(pkt.udp.dstport) in lwm2m_ports:
                if hasattr(pkt.udp, 'payload'):
                    payload = str(pkt.udp.payload)
                    
                    # Try to extract CoAP Message ID (bytes 2-3 in CoAP header)
                    if len(payload) >= 8:  # Minimum CoAP header
                        try:
                            # Convert hex payload to bytes and extract Message ID
                            hex_payload = payload.replace(':', '')
                            if len(hex_payload) >= 8:
                                msg_id = hex_payload[4:8]  # Message ID in CoAP header
                                
                                conn_id = f"{pkt.ip.dst}:{pkt.udp.dstport}"
                                message_signature = f"{conn_id}:{msg_id}"
                                
                                if message_signature in coap_messages:
                                    # Check time difference to avoid false positives
                                    time_diff = (pkt.sniff_time - coap_messages[message_signature]).total_seconds()
                                    if 0.1 < time_diff < 30:  # Reasonable retransmission window
                                        retransmissions += 1
                                else:
                                    coap_messages[message_signature] = pkt.sniff_time
                        except:
                            # Fallback to payload hash method
                            payload_hash = hash(payload[:50])  # First 50 chars only
                            conn_id = f"{pkt.ip.dst}:{pkt.udp.dstport}"
                            packet_signature = f"{conn_id}:{payload_hash}"
                            
                            if packet_signature in coap_messages:
                                retransmissions += 1
                            else:
                                coap_messages[packet_signature] = pkt.sniff_time
    
    return retransmissions

def calculate_dtls_handshake_time(packets):
    """Calculate DTLS handshake time for LwM2M"""
    lwm2m_ports = detect_lwm2m_ports(packets)
    
    if not lwm2m_ports:
        return None
    
    # For LwM2M, we'll measure the time between first and second response
    # as an approximation of DTLS handshake completion
    response_times = []
    
    for original_packet_num, pkt in packets:
        if hasattr(pkt, 'udp'):
            if hasattr(pkt.udp, 'srcport') and int(pkt.udp.srcport) in lwm2m_ports:
                response_times.append(pkt.sniff_time)
                if len(response_times) >= 2:
                    break
    
    if len(response_times) >= 2:
        return (response_times[1] - response_times[0]).total_seconds() * 1000  # Return in ms
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

def analyze_lwm2m_performance(packets, session_name):
    """Analyze LwM2M performance for a session - only 5 metrics"""
    print(f"  Analyzing {session_name} performance...")
    
    # Calculate only the 5 specified metrics
    metrics = {
        'latency': calculate_latency(packets),
        'retransmissions': calculate_retransmissions(packets),
        'dtls_handshake_time': calculate_dtls_handshake_time(packets),
        'rrc_state_changes': calculate_rrc_state_changes(packets),
        'network_registration_changes': calculate_network_registration_changes(packets)
    }
    
    return metrics

def process_measurement(measurement_path):
    """Process a single measurement folder"""
    pcap_file = os.path.join(measurement_path, 'lwm2m_trace.pcapng')
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
            json_key = 'lwm2m_data_ltem'
        elif session_name == 'NB-IoT':
            json_key = 'lwm2m_data_nbiot'
        else:
            continue
        
        # Create section if it doesn't exist
        if json_key not in data:
            data[json_key] = {}
        
        # Calculate only the 5 specified performance metrics
        metrics = analyze_lwm2m_performance(packets, session_name)
        
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
