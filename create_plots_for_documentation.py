import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import scipy.stats as stats
from scipy.stats import mannwhitneyu, kruskal
import warnings
import os
import json
from matplotlib.patches import Rectangle
warnings.filterwarnings('ignore')

band_params = {
    3:  {'N_off_DL': 1200, 'F_off_DL': 1805.0,
         'N_off_UL': 19200, 'F_off_UL': 1710.0},
    8:  {'N_off_DL': 3450, 'F_off_DL': 925.0,
         'N_off_UL': 18000, 'F_off_UL': 880.0},
    20: {'N_off_DL': 6150, 'F_off_DL': 791.0,
         'N_off_UL': 24150, 'F_off_UL': 832.0},
}

def get_data():
    data = []
    for file in os.listdir("Measurements/"):
        with open(os.path.join("Measurements/", file, "all_data.json"), 'r') as f:
            data.append(json.load(f))
    return data

def earfcn_to_dl_frequency(earfcn_dl, band):
    """
    Convert DL EARFCN to Downlink frequency (MHz).
    """
    if band not in band_params:
        raise ValueError(f"Band {band} not supported.")
    p = band_params[band]
    return p['F_off_DL'] + 0.1 * (earfcn_dl - p['N_off_DL'])

def earfcn_to_ul_frequency(earfcn_dl, band):
    """
    Convert DL EARFCN to Uplink frequency (MHz).
    Automatically calculates UL EARFCN from DL EARFCN.
    """
    if band not in band_params:
        raise ValueError(f"Band {band} not supported.")
    p = band_params[band]
    earfcn_ul = earfcn_dl + (p['N_off_UL'] - p['N_off_DL'])
    return p['F_off_UL'] + 0.1 * (earfcn_ul - p['N_off_UL'])

def convert_rsrp(raw):
    if raw is None:
        return None
    elif raw == -17:
        return -157
    elif -16 <= raw <= -1:
        return -156 + (raw + 16)
    elif raw == 0:
        return -141
    elif 1 <= raw <= 96:
        return -140 + (raw - 1)
    elif raw == 97:
        return -44
    else:
        return None

def convert_rsrq(raw):
    if raw is None:
        return None
    elif raw == -30:
        return -35
    elif -29 <= raw <= -1:
        return -34 + (raw + 30) * 0.5
    elif raw == 0:
        return -20
    elif 1 <= raw <= 45:
        return -19.5 + (raw - 1) * 0.5
    elif raw == 46:
        return 2.5
    else:
        return None

def convert_snr(raw):
    if raw is None:
        return None
    elif 0 <= raw <= 49:
        return raw - 24
    elif raw == 127:
        return None
    else:
        return None
    
def create_dataframe(data):
    # Extracting lwm2m_data_ltem data
    lat = [d['location_latitude'] for d in data]
    lon = [d['location_longitude'] for d in data]
    buffer = [data['lwm2m_data_ltem'] for data in data if 'lwm2m_data_ltem' in data]
    band = [d['Band'] for d in buffer]
    earfcn = [d['Earfcn'] for d in buffer]
    uplink_frequency = [earfcn_to_ul_frequency(d['Earfcn'], d['Band']) for d in buffer]
    downlink_frequency = [earfcn_to_dl_frequency(d['Earfcn'], d['Band']) for d in buffer]
    tx_power = [d['Tx Power'] for d in buffer]
    rx_repetitions = [d['Rx Repetitions'] for d in buffer]
    tx_repetitions = [d['Tx Repetitions'] for d in buffer]
    energy_estimate = [d['Energy Estimate'] for d in buffer]
    rsrp = [convert_rsrp(d['RSRP']) for d in buffer]
    rsrq = [convert_rsrq(d['RSRQ']) for d in buffer]
    snr = [convert_snr(d['SNR']) for d in buffer]
    dl_pathloss = [d['DL Pathloss'] for d in buffer]
    link = [d['Link'] for d in buffer]
    protocol = ["LWM2M" for d in buffer]
    link_standard = ["LTE-M" for d in buffer]
    connection_time_link = [d['Connection Time Link'] / 1000.0 for d in buffer]
    connection_time_protocol = [d['Connection Time Protocol'] / 1000.0 for d in buffer]
    throughput_link = [d['Link throughput'] / 1000.0 for d in buffer]
    latency = [d['latency'] / 1000.0 for d in buffer]
    retransmissions = [d['retransmissions'] for d in buffer]
    security_handshake_time = [d['dtls_handshake_time'] / 1000.0 for d in buffer]
    rrc_state_changes = [d['rrc_state_changes'] for d in buffer]
    network_registration_changes = [d['network_registration_changes'] for d in buffer]

    # Extracting lwm2m_data_nbiot data

    buffer = [data['lwm2m_data_nbiot'] for data in data if 'lwm2m_data_nbiot' in data]
    lat += [d['location_latitude'] for d in data]
    lon += [d['location_longitude'] for d in data]
    band += [d['Band'] for d in buffer]
    earfcn += [d['Earfcn'] for d in buffer]
    uplink_frequency += [earfcn_to_ul_frequency(d['Earfcn'], d['Band']) for d in buffer]
    downlink_frequency += [earfcn_to_dl_frequency(d['Earfcn'], d['Band']) for d in buffer]
    tx_power += [d['Tx Power'] for d in buffer]
    rx_repetitions += [d['Rx Repetitions'] for d in buffer]
    tx_repetitions += [d['Tx Repetitions'] for d in buffer]
    energy_estimate += [d['Energy Estimate'] for d in buffer]
    rsrp += [convert_rsrp(d['RSRP']) for d in buffer]
    rsrq += [convert_rsrq(d['RSRQ']) for d in buffer]
    snr += [convert_snr(d['SNR']) for d in buffer]
    dl_pathloss += [d['DL Pathloss'] for d in buffer]
    link += [d['Link'] for d in buffer]
    protocol += ["LWM2M" for d in buffer]
    link_standard += ["NB-IoT" for d in buffer]
    connection_time_link += [d['Connection Time Link'] / 1000.0 for d in buffer]
    connection_time_protocol += [d['Connection Time Protocol'] / 1000.0 for d in buffer]
    throughput_link += [d['Link throughput'] / 1000.0 for d in buffer]
    latency += [d['latency'] / 1000.0 for d in buffer]
    retransmissions += [d['retransmissions'] for d in buffer]
    security_handshake_time += [d['dtls_handshake_time'] / 1000.0 for d in buffer]
    rrc_state_changes += [d['rrc_state_changes'] for d in buffer]
    network_registration_changes += [d['network_registration_changes'] for d in buffer]

    # Extracting mqtt_data_ltem data
    buffer = [data['mqtt_data_ltem'] for data in data if 'mqtt_data_ltem' in data]
    lat += [d['location_latitude'] for d in data]
    lon += [d['location_longitude'] for d in data]
    band += [d['Band'] for d in buffer]
    earfcn += [d['Earfcn'] for d in buffer]
    uplink_frequency += [earfcn_to_ul_frequency(d['Earfcn'], d['Band']) for d in buffer]
    downlink_frequency += [earfcn_to_dl_frequency(d['Earfcn'], d['Band']) for d in buffer]
    tx_power += [d['Tx Power'] for d in buffer]
    rx_repetitions += [d['Rx Repetitions'] for d in buffer]
    tx_repetitions += [d['Tx Repetitions'] for d in buffer]
    energy_estimate += [d['Energy Estimate'] for d in buffer]
    rsrp += [convert_rsrp(d['RSRP']) for d in buffer]
    rsrq += [convert_rsrq(d['RSRQ']) for d in buffer]
    snr += [convert_snr(d['SNR']) for d in buffer]
    dl_pathloss += [d['DL Pathloss'] for d in buffer]
    link += [d['Link'] for d in buffer]
    protocol += ["MQTT" for d in buffer]
    link_standard += ["LTE-M" for d in buffer]
    connection_time_link += [d['Connection Time Link'] / 1000.0 for d in buffer]
    connection_time_protocol += [d['Connection Time Protocol'] / 1000.0 for d in buffer]
    throughput_link += [d['Link throughput'] / 1000.0 for d in buffer]
    latency += [d['latency'] / 1000.0 for d in buffer]
    retransmissions += [d['retransmissions'] for d in buffer]
    security_handshake_time += [d['tls_handshake_time'] / 1000.0 for d in buffer]
    rrc_state_changes += [d['rrc_state_changes'] for d in buffer]
    network_registration_changes += [d['network_registration_changes'] for d in buffer]


    # Extracting mqtt_data_nbiot data
    buffer = [data['mqtt_data_nbiot'] for data in data if 'mqtt_data_nbiot' in data]
    lat += [d['location_latitude'] for d in data]
    lon += [d['location_longitude'] for d in data]
    band += [d['Band'] for d in buffer]
    earfcn += [d['Earfcn'] for d in buffer]
    uplink_frequency += [earfcn_to_ul_frequency(d['Earfcn'], d['Band']) for d in buffer]
    downlink_frequency += [earfcn_to_dl_frequency(d['Earfcn'], d['Band']) for d in buffer]
    tx_power += [d['Tx Power'] for d in buffer]
    rx_repetitions += [d['Rx Repetitions'] for d in buffer]
    tx_repetitions += [d['Tx Repetitions'] for d in buffer]
    energy_estimate += [d['Energy Estimate'] for d in buffer]
    rsrp += [convert_rsrp(d['RSRP']) for d in buffer]
    rsrq += [convert_rsrq(d['RSRQ']) for d in buffer]
    snr += [convert_snr(d['SNR']) for d in buffer]
    dl_pathloss += [d['DL Pathloss'] for d in buffer]
    link += [d['Link'] for d in buffer]
    protocol += ["MQTT" for d in buffer]
    link_standard += ["NB-IoT" for d in buffer]
    connection_time_link += [d['Connection Time Link'] / 1000.0 for d in buffer]
    connection_time_protocol += [d['Connection Time Protocol'] / 1000.0 for d in buffer]
    throughput_link += [d['Link throughput'] / 1000.0 for d in buffer]
    latency += [d['latency'] / 1000.0 for d in buffer]
    retransmissions += [d['retransmissions'] for d in buffer]
    security_handshake_time += [d['tls_handshake_time'] / 1000.0 for d in buffer]
    rrc_state_changes += [d['rrc_state_changes'] for d in buffer]
    network_registration_changes += [d['network_registration_changes'] for d in buffer]


    df = pd.DataFrame({
        'lat': lat,
        'lon': lon,
        'Band': band,
        'EARFCN': earfcn,
        'TX power': tx_power,
        'RX repetitions': rx_repetitions,
        'TX repetitions': tx_repetitions,
        'Energy estimate': energy_estimate,
        'Uplink frequency': uplink_frequency,
        'Downlink frequency': downlink_frequency,
        'RSRP': rsrp,
        'RSRQ': rsrq,
        'SNR': snr,
        'Downlink pathloss': dl_pathloss,
        'Link technology': link,
        'Transport protocol': protocol,
        'Communication technology': link_standard,
        'Connection time (link)': connection_time_link,  # Convert to seconds
        'Connection time (protocol)': connection_time_protocol, # Convert to seconds
        'Throughput (link)': throughput_link, # Convert to kbps
        'Latency': latency,
        'Retransmissions': retransmissions,
        'Security handshake time': security_handshake_time,
        'RRC state changes': rrc_state_changes,
        'Network registration changes': network_registration_changes,
    })
    return df

def plot_statistics_kde_with_ci(data, data_name, filter_name, title, unit="", file_name="", save_path="Figures/", show_plot=False, confidence_level=0.95, custom_x_limits=None, custom_y_limits=None):
    """
    Plot KDE distribution with confidence intervals for two-group comparison
    
    Parameters:
    - data: DataFrame with the data
    - data_name: column name for the metric to plot
    - filter_name: column name for grouping (e.g., 'Communication technology')
    - title: title for the plots
    - unit: unit of measurement
    - file_name: name for saved file
    - save_path: path to save figures
    - show_plot: whether to display the plot
    - confidence_level: confidence level for CI (default 0.95)
    - custom_x_limits: tuple (x_min, x_max) for KDE plot X-axis limits
    - custom_y_limits: dict with 'kde': (y_min, y_max) and 'bar': (y_min, y_max) for Y-axis limits
    """
    import scipy.stats as stats
    
    fig, axes = plt.subplots(1, 2, figsize=(18, 7))
    
    categories = data[filter_name].unique()
    colors = ['#2E86C1', '#E74C3C']
    
    # First, calculate all statistics
    stats_summary = {}
    
    for i, category in enumerate(categories):
        category_data = data[data[filter_name] == category][data_name].dropna()
        
        if len(category_data) > 1:
            mean_val = category_data.mean()
            std_val = category_data.std()
            n = len(category_data)
            
            alpha = 1 - confidence_level
            t_critical = stats.t.ppf(1 - alpha/2, df=n-1)
            margin_error = t_critical * (std_val / np.sqrt(n))
            ci_lower = mean_val - margin_error
            ci_upper = mean_val + margin_error
            
            stats_summary[category] = {
                'mean': mean_val, 'std': std_val, 'n': n,
                'ci_lower': ci_lower, 'ci_upper': ci_upper,
                'margin_error': margin_error
            }
    
    # 1. KDE plot
    sns.kdeplot(data=data, x=data_name, hue=filter_name, fill=True, ax=axes[0], 
                linewidth=2, alpha=0.7, palette=colors)
    
    # Set CUSTOM X-axis limits for KDE plot
    if custom_x_limits is not None:
        axes[0].set_xlim(custom_x_limits[0], custom_x_limits[1])
        print(f"✅ Using custom X-axis limits for KDE: [{custom_x_limits[0]}, {custom_x_limits[1]}]")
    
    # Set CUSTOM Y-axis limits for KDE plot
    if custom_y_limits is not None and 'kde' in custom_y_limits:
        axes[0].set_ylim(custom_y_limits['kde'][0], custom_y_limits['kde'][1])
        print(f"✅ Using custom Y-axis limits for KDE: [{custom_y_limits['kde'][0]}, {custom_y_limits['kde'][1]}]")
    else:
        # Default Y-axis behavior
        y_min, y_max = axes[0].get_ylim()
        y_padding = y_max * 0.1
        axes[0].set_ylim(0, y_max + y_padding)
    
    # Add visual elements AFTER setting limits
    for i, category in enumerate(categories):
        if category in stats_summary:
            stats = stats_summary[category]
            
            # Add vertical line for mean
            axes[0].axvline(stats['mean'], color=colors[i], linestyle='--', linewidth=3, 
                           alpha=0.8, label=f'{category} Mean: {stats["mean"]:.3f}')
            
            # Add confidence interval as shaded area
            axes[0].axvspan(stats['ci_lower'], stats['ci_upper'], alpha=0.3, color=colors[i],
                           label=f'{category} {confidence_level*100:.0f}% CI')
    
    axes[0].set_title(f"Kernel Density Estimation for {title}\nby Technology with {confidence_level*100:.0f}% Confidence Interval", 
                     fontsize=20, fontweight='bold')
    axes[0].set_xlabel(f"{title} [{unit}]", fontsize=18, fontweight='bold')
    
    if unit:
        axes[0].set_ylabel(f'Probability Density [1/{unit}]', fontsize=18, fontweight='bold')
    else:
        axes[0].set_ylabel('Probability Density', fontsize=18, fontweight='bold')
    
    axes[0].grid(True, linestyle='--', alpha=0.8)
    axes[0].legend(fontsize=14, loc='best', prop={'weight': 'bold'})
    
    # 2. Bar chart
    if len(stats_summary) == 2:
        category_list = list(stats_summary.keys())
        means = [stats_summary[cat]['mean'] for cat in category_list]
        errors = [stats_summary[cat]['margin_error'] for cat in category_list]
        ns = [stats_summary[cat]['n'] for cat in category_list]
        
        x_pos = np.arange(len(category_list))
        bars = axes[1].bar(x_pos, means, yerr=errors, capsize=10, 
                          color=colors[:len(category_list)], alpha=0.8, 
                          edgecolor='black', linewidth=1.5)
        
        # Calculate label positions
        label_heights = []
        for i, (bar, mean_val, error, n) in enumerate(zip(bars, means, errors, ns)):
            label_y = bar.get_height() + error + abs(bar.get_height()) * 0.05
            label_heights.append(label_y)
            axes[1].text(bar.get_x() + bar.get_width()/2, label_y,
                        f'{mean_val:.3f} ± {error:.3f}\n(n={n})',
                        ha='center', va='bottom', fontsize=12, fontweight='bold')
        
        # Statistical test
        cat1, cat2 = category_list
        data1 = data[data[filter_name] == cat1][data_name].dropna()
        data2 = data[data[filter_name] == cat2][data_name].dropna()
        
        if len(data1) > 0 and len(data2) > 0:
            from scipy.stats import mannwhitneyu
            statistic, p_value = mannwhitneyu(data1, data2, alternative='two-sided')
            significance = "***" if p_value < 0.001 else "**" if p_value < 0.01 else "*" if p_value < 0.05 else "ns"
            
            axes[1].text(0.5, 0.95, f'p = {p_value:.4f} {significance}', 
                        transform=axes[1].transAxes, ha='center', va='top', fontsize=16,
                        bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        # Set X-axis limits for bar chart
        axes[1].set_xlim(-0.5, len(category_list) - 0.5)
        
        # Set CUSTOM Y-axis limits for bar chart
        if custom_y_limits is not None and 'bar' in custom_y_limits:
            axes[1].set_ylim(custom_y_limits['bar'][0], custom_y_limits['bar'][1])
            print(f"✅ Using custom Y-axis limits for bar chart: [{custom_y_limits['bar'][0]}, {custom_y_limits['bar'][1]}]")
        else:
            # Default Y-axis behavior for bar chart
            bar_bottoms = [mean - error for mean, error in zip(means, errors)]
            y_min_bar = min(bar_bottoms)
            y_max_bar = max(label_heights)
            
            y_range_bar = y_max_bar - y_min_bar
            y_padding_bottom = y_range_bar * 0.1 if y_min_bar < 0 else 0
            y_padding_top = y_range_bar * 0.1
            
            axes[1].set_ylim(y_min_bar - y_padding_bottom, y_max_bar + y_padding_top)
        
        axes[1].set_title(f"{title}: Technology Comparison\nwith {confidence_level*100:.0f}% Confidence Intervals", 
                     fontsize=20, fontweight='bold')
        axes[1].set_ylabel(f"{title} [{unit}]", fontsize=18, fontweight='bold')
        axes[1].set_xlabel(f"{filter_name}", fontsize=18, fontweight='bold')
        axes[1].set_xticks(x_pos)
        axes[1].set_xticklabels(category_list, fontweight='bold')
        axes[1].grid(True, linestyle='--', alpha=0.8, axis='y')
    
    for ax in axes:
        ax.tick_params(labelsize=16)
        # Set y-tick labels bold
        ylabels = [label.get_text() for label in ax.get_yticklabels()]
        ax.set_yticklabels(ylabels, fontweight='bold')
        # Set x-tick labels bold
        xlabels = [label.get_text() for label in ax.get_xticklabels()]
        ax.set_xticklabels(xlabels, fontweight='bold')
    
    plt.tight_layout(pad=3.0)
    plt.savefig(f'{save_path}/{file_name}_kde_ci.png', dpi=300, 
                bbox_inches='tight', pad_inches=0.3,
                facecolor='white', edgecolor='none')
    
    if show_plot:
        plt.show()
    else:
        plt.close()

def create_radar_chart(df, metrics=None, group_by='Communication technology', save_path='Figures/', custom_ranges=None, show_plot=False):
    """Create radar chart for technology comparison with specific metrics"""
    from math import pi
    
    # Use your specific requested metrics if none provided (frequencies removed)
    if metrics is None:
        metrics = [
            'TX power', 'RSRQ', 'RSRP', 'SNR', 'Connection time (link)', 'Energy estimate',
            'TX repetitions', 'RX repetitions', 'Throughput (link)', 'Downlink pathloss'
        ]
    
    # Filter available metrics from dataframe
    available_metrics = [m for m in metrics if m in df.columns]

    if len(available_metrics) < 3:
        print(f"⚠️ Not enough metrics available for radar plot. Found: {available_metrics}")
        return
    
    # Calculate percentile-based ranges for each metric across ALL data (LTE-M + NB-IoT)
    # Using 5th and 95th percentiles to avoid outlier influence
    global_ranges = {}
    for metric in available_metrics:
        all_data = df[metric].dropna()
        if len(all_data) > 0:
            p5 = all_data.quantile(0.05)   # 5th percentile
            p95 = all_data.quantile(0.95)  # 95th percentile
            global_ranges[metric] = (p5, p95)
            print(f"📊 Percentile range for {metric}: [{p5:.3f}, {p95:.3f}] (5th-95th percentile)")
        else:
            print(f"⚠️ No data found for {metric}")
    
    # Separate metrics by performance direction
    lower_is_better = ['TX power', 'Connection time (link)', 'Energy estimate', 
                      'TX repetitions', 'RX repetitions', 'Downlink pathloss']
    higher_is_better = ['RSRQ', 'RSRP', 'SNR', 'Throughput (link)']
    
    # Split metrics into two groups
    lower_metrics = [m for m in available_metrics if m in lower_is_better]
    higher_metrics = [m for m in available_metrics if m in higher_is_better]
    
    print(f"📈 Higher is better metrics: {higher_metrics}")
    print(f"📉 Lower is better metrics: {lower_metrics}")
    
    # Split data by Communication technology (LTE-M vs NB-IoT)
    ltem_data = df[df['Communication technology'] == 'LTE-M']
    nbiot_data = df[df['Communication technology'] == 'NB-IoT']

    if len(ltem_data) == 0 or len(nbiot_data) == 0:
        print("⚠️ Missing data for one or both technologies")
        return
    
    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10), subplot_kw=dict(projection='polar'))
    
    # Colors for each technology
    colors = {'LTE-M': '#2E86C1', 'NB-IoT': '#E74C3C'}
    
    def plot_radar_for_metrics(ax, metrics_list, title_suffix):
        if not metrics_list:
            ax.set_visible(False)
            return
        
        # Calculate mean values for each technology and metric
        tech_values = {'LTE-M': [], 'NB-IoT': []}
        
        for metric in metrics_list:
            # LTE-M values
            ltem_values = ltem_data[metric].dropna()
            if len(ltem_values) > 0:
                tech_values['LTE-M'].append(ltem_values.mean())
            else:
                tech_values['LTE-M'].append(0)
            
            # NB-IoT values  
            nbiot_values = nbiot_data[metric].dropna()
            if len(nbiot_values) > 0:
                tech_values['NB-IoT'].append(nbiot_values.mean())
            else:
                tech_values['NB-IoT'].append(0)
        
        # Normalize values to 0-1 scale using PERCENTILE RANGES from all data
        # NO INVERSION - direct normalization for both charts
        normalized_values = {'LTE-M': [], 'NB-IoT': []}
        
        for i, metric in enumerate(metrics_list):
            ltem_val = tech_values['LTE-M'][i]
            nbiot_val = tech_values['NB-IoT'][i]
            
            # Use percentile ranges (5th-95th percentile from entire dataset)
            if metric in global_ranges:
                p5, p95 = global_ranges[metric]
                print(f"✅ Using percentile range for {metric}: [{p5:.3f}, {p95:.3f}]")
                print(f"   LTE-M mean: {ltem_val:.3f}, NB-IoT mean: {nbiot_val:.3f}")
            else:
                print(f"⚠️ No percentile range found for {metric}")
                continue
            
            if p95 == p5:
                # If no variation, set both to middle
                normalized_values['LTE-M'].append(0.5)
                normalized_values['NB-IoT'].append(0.5)
                print(f"   No variation in {metric}, setting both to 0.5")
            else:
                # Direct normalization: (value - p5) / (p95 - p5)
                # Higher values = further from center (regardless of whether higher/lower is better)
                ltem_clamped = max(p5, min(p95, ltem_val))
                nbiot_clamped = max(p5, min(p95, nbiot_val))
                
                ltem_normalized = (ltem_clamped - p5) / (p95 - p5)
                nbiot_normalized = (nbiot_clamped - p5) / (p95 - p5)
                
                normalized_values['LTE-M'].append(ltem_normalized)
                normalized_values['NB-IoT'].append(nbiot_normalized)
                print(f"   Direct - LTE-M: {ltem_normalized:.3f}, NB-IoT: {nbiot_normalized:.3f}")
        
        # Set up the radar chart
        N = len(metrics_list)
        angles = [n / float(N) * 2 * pi for n in range(N)]
        angles += angles[:1]  # Complete the circle
        
        # Plot each technology
        for tech in ['LTE-M', 'NB-IoT']:
            values = normalized_values[tech]
            values += values[:1]  # Complete the circle
            
            ax.plot(angles, values, 'o-', linewidth=8, label=tech, 
                    color=colors[tech], markersize=6)
            ax.fill(angles, values, alpha=0.3, color=colors[tech])
        
        # Customize the plot
        ax.set_xticks(angles[:-1])
        ax.spines['polar'].set_linewidth(2)
        
        # Clean up metric labels for display
        display_labels = []
        for label in metrics_list:
            if label == 'TX power':
                display_labels.append('TX Power\n[dBm]')
            elif label == 'RSRQ':
                display_labels.append('RSRQ\n[dB]')
            elif label == 'RSRP':
                display_labels.append('RSRP\n[dBm]')
            elif label == 'SNR':
                display_labels.append('SNR\n[dB]')
            elif label == 'Connection time (link)':
                display_labels.append('Connection Time\n[s]')
            elif label == 'Energy estimate':
                display_labels.append('Energy Estimate\n[relative]')
            elif label == 'TX repetitions':
                display_labels.append('TX Repetitions\n[count]')
            elif label == 'RX repetitions':
                display_labels.append('RX Repetitions\n[count]')
            elif label == 'Throughput (link)':
                display_labels.append('Throughput\n[kbps]')
            elif label == 'Downlink pathloss':
                display_labels.append('DL Pathloss\n[dB]')
            else:
                display_labels.append(label)
        
        ax.set_xticklabels(display_labels, fontsize=16, fontweight='bold')
        ax.set_ylim(0, 1)
        
        # Add grid lines
        ax.set_yticks([0.2, 0.4, 0.6, 0.8, 1.0])
        ax.set_yticklabels(['0.2', '0.4', '0.6', '0.8', '1.0'], fontsize=16, fontweight='bold')
        ax.grid(True, alpha=0.6)
        
        # Add title and legend
        ax.set_title(f'LTE-M vs NB-IoT\n{title_suffix}', 
                    fontweight='bold', fontsize=20, pad=20)
        ax.legend(loc='upper right', bbox_to_anchor=(1.2, 1.0), fontsize=16, prop={'weight': 'bold'})

    # Plot both radar charts WITHOUT inversion
    plot_radar_for_metrics(ax1, higher_metrics, '(Higher values = Better Performance)')
    plot_radar_for_metrics(ax2, lower_metrics, '(Lower values = Better Performance)')
    
    plt.tight_layout()
    plt.savefig(f'{save_path}/technology_radar_comparison.png', dpi=300, bbox_inches='tight')
    
    if show_plot:
        plt.show()
    else:
        plt.close()
    
    print(f"✅ Radar chart saved to: {save_path}/technology_radar_comparison.png")

def plot_protocol_technology_kde(data, data_name, title, unit="", file_name="", save_path="Figures/", show_plot=False, custom_x_limits=None, custom_y_limits=None):
    """
    Plot KDE distribution comparing all protocol-technology combinations
    
    Parameters:
    - data: DataFrame with the data
    - data_name: column name for the metric to plot (e.g., 'Latency')
    - title: title for the plot
    - unit: unit of measurement
    - file_name: name for saved file
    - save_path: path to save figures
    - show_plot: whether to display the plot
    - custom_x_limits: tuple (x_min, x_max) for X-axis limits
    - custom_y_limits: tuple (y_min, y_max) for Y-axis limits
    """
    
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    
    # Define the 4 combinations
    combinations = [
        ('MQTT', 'NB-IoT'),
        ('MQTT', 'LTE-M'),
        ('LWM2M', 'NB-IoT'),
        ('LWM2M', 'LTE-M')
    ]
    
    # Define colors for each combination
    colors = ['#E74C3C', '#3498DB', '#F39C12', '#2ECC71']  # Red, Blue, Orange, Green
    
    # Plot KDE for each combination
    for i, (protocol, tech) in enumerate(combinations):
        # Filter data for this specific combination
        filtered_data = data[
            (data['Transport protocol'] == protocol) & 
            (data['Communication technology'] == tech)
        ][data_name].dropna()
        
        if len(filtered_data) > 0:
            # Create label for this combination
            label = f'{protocol} + {tech} (n={len(filtered_data)})'
            
            # Plot KDE - FIXED: removed alpha_fill parameter
            sns.kdeplot(data=filtered_data, ax=ax, color=colors[i], 
                       linewidth=3, alpha=0.3, label=label, fill=True)
            
            # Calculate and add mean line
            mean_val = filtered_data.mean()
            ax.axvline(mean_val, color=colors[i], linestyle='--', linewidth=2, 
                      alpha=0.9, label=f'{protocol}+{tech} Mean: {mean_val:.3f}')
        else:
            print(f"⚠️ No data found for {protocol} + {tech}")
    
    # Set CUSTOM X-axis limits
    if custom_x_limits is not None:
        ax.set_xlim(custom_x_limits[0], custom_x_limits[1])
        print(f"✅ Using custom X-axis limits: [{custom_x_limits[0]}, {custom_x_limits[1]}]")
    
    # Set CUSTOM Y-axis limits
    if custom_y_limits is not None:
        ax.set_ylim(custom_y_limits[0], custom_y_limits[1])
        print(f"✅ Using custom Y-axis limits: [{custom_y_limits[0]}, {custom_y_limits[1]}]")
    else:
        # Default Y-axis behavior
        y_min, y_max = ax.get_ylim()
        y_padding = y_max * 0.1
        ax.set_ylim(0, y_max + y_padding)
    
    # Customize the plot
    ax.set_title(f"Kernel Density Estimation for {title}\nby Protocol and Technology", 
                fontsize=20, fontweight='bold')
    ax.set_xlabel(f"{title} [{unit}]", fontsize=18, fontweight='bold')

    if unit:
        ax.set_ylabel(f'Probability Density [1/{unit}]', fontsize=18, fontweight='bold')
    else:
        ax.set_ylabel('Probability Density', fontsize=18, fontweight='bold')

# ...existing code...
    ax.grid(True, linestyle='--', alpha=0.8)
    ax.legend(fontsize=14, loc='best', prop={'weight': 'bold'})
    ax.tick_params(labelsize=16)

    # Make x-tick labels bold
    xlabels = [label.get_text() for label in ax.get_xticklabels()]
    ax.set_xticklabels(xlabels, fontweight='bold', fontsize=9)

    # Make y-tick labels bold
    ylabels = [label.get_text() for label in ax.get_yticklabels()]
    ax.set_yticklabels(ylabels, fontweight='bold', fontsize=9)

    plt.tight_layout(pad=3.0)
    plt.savefig(f'{save_path}/{file_name}_protocol_tech_kde.png', dpi=300, 
                bbox_inches='tight', pad_inches=0.3,
                facecolor='white', edgecolor='none')
    
    if show_plot:
        plt.show()
    else:
        plt.close()

def plot_protocol_technology_ci(data, data_name, title, unit="", file_name="", save_path="Figures/", show_plot=False, confidence_level=0.95, custom_x_limits=None, custom_y_limits=None):
    """
    Plot bar chart with confidence intervals comparing all protocol-technology combinations
    
    Parameters:
    - data: DataFrame with the data
    - data_name: column name for the metric to plot (e.g., 'Latency')
    - title: title for the plot
    - unit: unit of measurement
    - file_name: name for saved file
    - save_path: path to save figures
    - show_plot: whether to display the plot
    - confidence_level: confidence level for CI (default 0.95)
    - custom_x_limits: Not used for bar chart (categories on X-axis)
    - custom_y_limits: tuple (y_min, y_max) for Y-axis limits
    """
    import scipy.stats as stats
    
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    
    # Define the 4 combinations
    combinations = [
        ('MQTT', 'NB-IoT'),
        ('MQTT', 'LTE-M'),
        ('LWM2M', 'NB-IoT'),
        ('LWM2M', 'LTE-M')
    ]
    
    # Define colors for each combination
    colors = ['#E74C3C', '#3498DB', '#F39C12', '#2ECC71']  # Red, Blue, Orange, Green
    
    # Calculate statistics for each combination
    stats_summary = {}
    means = []
    errors = []
    labels = []
    bar_colors = []
    ns = []
    
    for i, (protocol, tech) in enumerate(combinations):
        # Filter data for this specific combination
        filtered_data = data[
            (data['Transport protocol'] == protocol) & 
            (data['Communication technology'] == tech)
        ][data_name].dropna()
        
        if len(filtered_data) > 1:
            mean_val = filtered_data.mean()
            std_val = filtered_data.std()
            n = len(filtered_data)
            
            # Calculate confidence interval
            alpha = 1 - confidence_level
            t_critical = stats.t.ppf(1 - alpha/2, df=n-1)
            margin_error = t_critical * (std_val / np.sqrt(n))
            ci_lower = mean_val - margin_error
            ci_upper = mean_val + margin_error
            
            stats_summary[f'{protocol}+{tech}'] = {
                'mean': mean_val, 'std': std_val, 'n': n,
                'ci_lower': ci_lower, 'ci_upper': ci_upper,
                'margin_error': margin_error
            }
            
            means.append(mean_val)
            errors.append(margin_error)
            labels.append(f'{protocol}\n+\n{tech}')
            bar_colors.append(colors[i])
            ns.append(n)
            
        elif len(filtered_data) == 1:
            # Single data point - no CI possible
            mean_val = filtered_data.iloc[0]
            means.append(mean_val)
            errors.append(0)
            labels.append(f'{protocol}\n+\n{tech}')
            bar_colors.append(colors[i])
            ns.append(1)
            print(f"⚠️ Only 1 data point for {protocol} + {tech}, no CI calculated")
        else:
            print(f"⚠️ No data found for {protocol} + {tech}")
    
    if len(means) == 0:
        print("❌ No data available for any combination")
        return
    
    # Create bar chart
    x_pos = np.arange(len(labels))
    bars = ax.bar(x_pos, means, yerr=errors, capsize=10, 
                color=bar_colors, alpha=0.8, 
                edgecolor='black', linewidth=1.5)
    
    # Calculate label positions
    label_heights = []
    for i, (bar, mean_val, error, n) in enumerate(zip(bars, means, errors, ns)):
        label_y = bar.get_height() + error + abs(bar.get_height()) * 0.05
        label_heights.append(label_y)
        ax.text(bar.get_x() + bar.get_width()/2, label_y,
                f'{mean_val:.3f} ± {error:.3f}\n(n={n})',
                ha='center', va='bottom', fontsize=16, fontweight='bold')
    
    # Perform statistical tests if we have multiple groups
    if len(means) > 1:
        # Get all data for Kruskal-Wallis test (non-parametric ANOVA)
        all_groups = []
        group_names = []
        for protocol, tech in combinations:
            filtered_data = data[
                (data['Transport protocol'] == protocol) & 
                (data['Communication technology'] == tech)
            ][data_name].dropna()
            
            if len(filtered_data) > 0:
                all_groups.append(filtered_data.values)
                group_names.append(f'{protocol}+{tech}')
        
        if len(all_groups) > 1:
            try:
                from scipy.stats import kruskal
                statistic, p_value = kruskal(*all_groups)
                significance = "***" if p_value < 0.001 else "**" if p_value < 0.01 else "*" if p_value < 0.05 else "ns"
                
                ax.text(0.5, 0.95, f'Kruskal-Wallis: p = {p_value:.4f} {significance}', 
                        transform=ax.transAxes, ha='center', va='top', fontsize=16,
                        bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
            except:
                print("⚠️ Could not perform statistical test")
    
    # Set CUSTOM Y-axis limits
    if custom_y_limits is not None:
        ax.set_ylim(custom_y_limits[0], custom_y_limits[1])
        print(f"✅ Using custom Y-axis limits: [{custom_y_limits[0]}, {custom_y_limits[1]}]")
    else:
        # Default Y-axis behavior
        if label_heights:
            bar_bottoms = [mean - error for mean, error in zip(means, errors)]
            y_min_bar = min(bar_bottoms)
            y_max_bar = max(label_heights)
            
            y_range_bar = y_max_bar - y_min_bar
            y_padding_bottom = y_range_bar * 0.1 if y_min_bar < 0 else 0
            y_padding_top = y_range_bar * 0.1
            
            ax.set_ylim(y_min_bar - y_padding_bottom, y_max_bar + y_padding_top)
    
    # Customize the plot
    ax.set_title(f"{title}: Comparison by Protocol and Technology\nwith {confidence_level*100:.0f}% Confidence Intervals", 
                fontsize=20, fontweight='bold')
    ax.set_ylabel(f"{title} [{unit}]", fontsize=18, fontweight='bold')
    ax.set_xlabel("Protocol + Communication Technology", fontsize=18, fontweight='bold')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels, fontweight='bold', fontsize=16)
    ax.grid(True, linestyle='--', alpha=0.8, axis='y')
    ax.tick_params(labelsize=16)

    # Make y-tick labels bold
    ylabels = [label.get_text() for label in ax.get_yticklabels()]
    ax.set_yticklabels(ylabels, fontweight='bold', fontsize=16)
    
    plt.tight_layout(pad=3.0)
    plt.savefig(f'{save_path}/{file_name}_protocol_tech_ci.png', dpi=300, 
                bbox_inches='tight', pad_inches=0.3,
                facecolor='white', edgecolor='none')
    
    if show_plot:
        plt.show()
    else:
        plt.close()

def plot_frequency_kde(data, data_name, filter_name, title, unit="", file_name="", save_path="Figures/", show_plot=False, custom_x_limits=None, custom_y_limits=None):
    """
    Plot KDE distribution for frequency data (no confidence intervals needed)
    
    Parameters:
    - data: DataFrame with the data
    - data_name: column name for the metric to plot
    - filter_name: column name for grouping (e.g., 'Communication technology')
    - title: title for the plot
    - unit: unit of measurement
    - file_name: name for saved file
    - save_path: path to save figures
    - show_plot: whether to display the plot
    - custom_x_limits: tuple (x_min, x_max) for X-axis limits
    - custom_y_limits: tuple (y_min, y_max) for Y-axis limits
    """
    
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    
    categories = data[filter_name].unique()
    colors = ['#2E86C1', '#E74C3C']
    
    # Plot KDE with explicit legend labels
    for i, category in enumerate(categories):
        category_data = data[data[filter_name] == category][data_name].dropna()
        if len(category_data) > 0:
            sns.kdeplot(data=category_data, ax=ax, color=colors[i], 
                       linewidth=2, alpha=0.7, fill=True, label=category)
    
    # Set CUSTOM X-axis limits
    if custom_x_limits is not None:
        ax.set_xlim(custom_x_limits[0], custom_x_limits[1])
        print(f"✅ Using custom X-axis limits: [{custom_x_limits[0]}, {custom_x_limits[1]}]")
    
    # Set CUSTOM Y-axis limits
    if custom_y_limits is not None:
        ax.set_ylim(custom_y_limits[0], custom_y_limits[1])
        print(f"✅ Using custom Y-axis limits: [{custom_y_limits[0]}, {custom_y_limits[1]}]")
    else:
        # Default Y-axis behavior
        y_min, y_max = ax.get_ylim()
        y_padding = y_max * 0.1
        ax.set_ylim(0, y_max + y_padding)
    
    ax.set_title(f"Kernel Density Estimation for {title}\nby Technology", 
                fontsize=20, fontweight='bold')
    ax.set_xlabel(f"{title} [{unit}]", fontsize=18, fontweight='bold')
    ax.set_ylabel(f'Probability Density [1/{unit}]', fontsize=18, fontweight='bold')

    ax.grid(True, linestyle='--', alpha=0.8)
    ax.legend(fontsize=12, loc='best', prop={'weight': 'bold'})
    ax.tick_params(labelsize=16)

    # Make x-tick labels bold
    xlabels = [label.get_text() for label in ax.get_xticklabels()]
    ax.set_xticklabels(xlabels, fontweight='bold')

    # Make y-tick labels bold
    ylabels = [label.get_text() for label in ax.get_yticklabels()]
    ax.set_yticklabels(ylabels, fontweight='bold')

    plt.tight_layout(pad=3.0)
    plt.savefig(f'{save_path}/{file_name}_frequency_kde.png', dpi=300, 
                bbox_inches='tight', pad_inches=0.3,
                facecolor='white', edgecolor='none')
    
    if show_plot:
        plt.show()
    else:
        plt.close()

def create_communication_technology_table(df, save_path="Figures/", file_name="communication_technology_comparison", 
                                         show_plot=True, dpi=900):
    """
    Create a comparison table for communication technologies (NB-IoT vs LTE-M)
    
    Parameters:
    - df: DataFrame with your measurement data
    - save_path: path to save the figure
    - file_name: name for the saved file
    - show_plot: whether to display the plot
    - dpi: resolution for saved figure
    """
    
    # Calculate statistics for each combination
    def calculate_stats(data, metric):
        """Calculate mean, std, and sample size for a metric"""
        clean_data = data[metric].dropna()
        if len(clean_data) > 0:
            return {
                'mean': clean_data.mean(),
                'std': clean_data.std(),
                'n': len(clean_data),
                'median': clean_data.median()
            }
        return {'mean': np.nan, 'std': np.nan, 'n': 0, 'median': np.nan}
    
    # Get data for each technology
    ltem_data = df[df['Communication technology'] == 'LTE-M']
    nbiot_data = df[df['Communication technology'] == 'NB-IoT']
    
    # Define metrics and their properties
    comm_metrics = [
        ('TX power', 'dBm', 'lower'),
        ('RSRP', 'dBm', 'higher'),
        ('RSRQ', 'dB', 'higher'),
        ('SNR', 'dB', 'higher'),
        ('Connection time (link)', 's', 'lower'),
        ('Throughput (link)', 'kbps', 'higher'),
        ('Energy estimate', 'relative', 'lower'),
        ('TX repetitions', 'count', 'lower'),
        ('RX repetitions', 'count', 'lower'),
        ('Downlink pathloss', 'dB', 'lower'),
    ]
    
    # Create the figure
    fig, ax = plt.subplots(1, 1, figsize=(32, 24))
    # fig.suptitle('Communication Technology Performance Comparison', 
    #              fontsize=20, fontweight='bold', y=0.95)
    
    # Add extra space at bottom for the lowest row
    ax.set_xlim(0, 10)
    ax.set_ylim(-0.5, len(comm_metrics) + 2)
    ax.axis('off')
    
    # Title
    ax.text(5, len(comm_metrics) + 1.5, 'NB-IoT vs LTE-M Comparison', ha='center', va='center', 
            fontsize=40, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.8))
    
    # Headers
    headers = ['Metric', 'NB-IoT', 'LTE-M', 'Winner', 'Key Insight']
    col_widths = [2.5, 1.8, 1.8, 1.2, 2.7]
    
    x_positions = [0]
    for width in col_widths[:-1]:
        x_positions.append(x_positions[-1] + width)
    
    # Draw header
    y_pos = len(comm_metrics) + 0.5
    for i, (header, x_pos, width) in enumerate(zip(headers, x_positions, col_widths)):
        rect = Rectangle((x_pos, y_pos - 0.25), width, 0.5, 
                       facecolor='darkblue', alpha=0.8)
        ax.add_patch(rect)
        ax.text(x_pos + width/2, y_pos, header, ha='center', va='center',
               fontweight='bold', color='white', fontsize=32)
    
    # Process each metric
    for i, (metric, unit, better_direction) in enumerate(comm_metrics):
        y_pos = len(comm_metrics) - i - 0.5
        
        if metric not in df.columns:
            continue
        
        # Calculate statistics
        stats1 = calculate_stats(nbiot_data, metric)
        stats2 = calculate_stats(ltem_data, metric)
        labels = ['NB-IoT', 'LTE-M']

        
        
        # Determine winner
        if not np.isnan(stats1['mean']) and not np.isnan(stats2['mean']):
            if better_direction == 'lower':
                winner = labels[0] if stats1['mean'] < stats2['mean'] else labels[1]
            else:
                winner = labels[0] if stats1['mean'] > stats2['mean'] else labels[1]
            # Set winner color based on label
            if winner.lower() in ['nb-iot', 'nbiot']:
                winner_color = '#d62728'  # NB-IoT red
            elif winner.lower() in ['lte-m', 'ltem', 'lte m']:
                winner_color = '#1f77b4'  # LTE-M blue
            else:
                winner_color = 'lightgreen'
        else:
            winner = 'N/A'
            winner_color = 'lightgray'
        
        # Generate insight
        insights = {
            'TX power': 'Energy efficiency vs coverage',
            'RSRP': 'Signal strength & coverage',
            'RSRQ': 'Signal quality & interference',
            'SNR': 'Noise immunity & robustness',
            'Connection time (link)': 'Network responsiveness',
            'Throughput (link)': 'Data transmission speed',
            'Energy estimate': 'Battery life impact',
            'TX repetitions': 'Coverage enhancement cost',
            'RX repetitions': 'Reception reliability cost',
            'Downlink pathloss': 'Deep coverage capability',
        }
        
        # Draw row
        colors = ['white', 'lightgray']
        row_color = colors[i % 2]
        
        # Data values
        values = [
            f"{stats1['mean']:.2f}±{stats1['std']:.2f} (n={stats1['n']})" if not np.isnan(stats1['mean']) else 'N/A',
            f"{stats2['mean']:.2f}±{stats2['std']:.2f} (n={stats2['n']})" if not np.isnan(stats2['mean']) else 'N/A'
        ]
        
        row_data = [f"{metric}\n[{unit}]"] + values + [winner, insights.get(metric, 'Performance trade-off')]
        
        for j, (data_item, x_pos, width) in enumerate(zip(row_data, x_positions, col_widths)):
            # Color winner column
            cell_color = winner_color if j == 3 and winner != 'N/A' else row_color
            
            rect = Rectangle((x_pos, y_pos - 0.4), width, 0.8, 
                           facecolor=cell_color, alpha=0.6, edgecolor='black')
            ax.add_patch(rect)
            
            # Adjust font size based on content length
            font_size = 8 if len(str(data_item)) > 25 else 9
            ax.text(x_pos + width/2, y_pos, str(data_item), ha='center', va='center', fontweight='bold',
                   fontsize=30, wrap=True)
    
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.05)
    plt.savefig(f'{save_path}/{file_name}.png', dpi=dpi, bbox_inches='tight', 
                facecolor='white', edgecolor='none')
    
    if show_plot:
        plt.show()
    else:
        plt.close()
    
    print(f"✅ Communication technology comparison table saved to: {save_path}/{file_name}.png")

def create_4way_comparison_table(df, save_path="Figures/", file_name="4way_protocol_technology_comparison", 
                                show_plot=True, dpi=900):
    """
    Create a comprehensive 4-way comparison table for all protocol-technology combinations
    focusing on protocol-relevant metrics only
    
    Parameters:
    - df: DataFrame with your measurement data
    - save_path: path to save the figure
    - file_name: name for the saved file
    - show_plot: whether to display the plot
    - dpi: resolution for saved figure
    """
    
    # Calculate statistics for each combination
    def calculate_stats(data, metric):
        """Calculate mean, std, and sample size for a metric"""
        clean_data = data[metric].dropna()
        if len(clean_data) > 0:
            return {
                'mean': clean_data.mean(),
                'std': clean_data.std(),
                'n': len(clean_data),
                'median': clean_data.median()
            }
        return {'mean': np.nan, 'std': np.nan, 'n': 0, 'median': np.nan}
    
    # Get data for each combination
    combinations = [
        ('MQTT', 'NB-IoT'),
        ('MQTT', 'LTE-M'),
        ('LWM2M', 'NB-IoT'),
        ('LWM2M', 'LTE-M')
    ]
    
    combination_data = {}
    for protocol, tech in combinations:
        combination_data[f'{protocol}+{tech}'] = df[
            (df['Transport protocol'] == protocol) & 
            (df['Communication technology'] == tech)
        ]
    
    # Define ONLY protocol-relevant metrics (removed network parameters)
    protocol_metrics = [
        # Protocol-specific performance metrics
        ('Connection time (protocol)', 's', 'lower'),
        ('Latency', 's', 'lower'),
        ('Security handshake time', 's', 'lower'),
        ('Retransmissions', 'count', 'lower'),
        ('RRC state changes', 'count', 'lower'),
        ('Network registration changes', 'count', 'lower'),
    ]
    
    # Create the figure - OPTIMIZED for fewer rows
    fig, ax = plt.subplots(1, 1, figsize=(32, 20))  # Reduced height for fewer metrics
    
    # TIGHT axis limits - minimal blank space
    ax.set_xlim(0, 10.4)
    ax.set_ylim(-0.3, len(protocol_metrics) + 2.5)
    ax.axis('off')
    
    # Title
    ax.text(5.2, len(protocol_metrics) + 1.8, 'Protocol Performance Comparison Matrix', 
            ha='center', va='center', fontsize=35, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='lightsteelblue', alpha=0.9))
    
    # Headers
    headers = ['Metric', 'MQTT+NB-IoT', 'MQTT+LTE-M', 'LWM2M+NB-IoT', 'LWM2M+LTE-M', 'Key Insight']
    col_widths = [1.8, 1.6, 1.6, 1.6, 1.6, 2.2]
    
    x_positions = [0]
    for width in col_widths[:-1]:
        x_positions.append(x_positions[-1] + width)
    
    # Define colors for each combination
    combo_colors = {
        'MQTT+NB-IoT': '#E74C3C',      # Red
        'MQTT+LTE-M': '#3498DB',       # Blue  
        'LWM2M+NB-IoT': '#F39C12',     # Orange
        'LWM2M+LTE-M': '#2ECC71'       # Green
    }
    
    # Draw header
    y_pos = len(protocol_metrics) + 1
    header_colors = ['darkslategray', '#E74C3C', '#3498DB', '#F39C12', '#2ECC71', 'darkslategray']
    
    for i, (header, x_pos, width, color) in enumerate(zip(headers, x_positions, col_widths, header_colors)):
        rect = Rectangle((x_pos, y_pos - 0.25), width, 0.5,
                       facecolor=color, alpha=0.9)
        ax.add_patch(rect)
        ax.text(x_pos + width/2, y_pos, header, ha='center', va='center',
               fontweight='bold', color='white', fontsize=32)
    
    # Process each metric
    for i, (metric, unit, better_direction) in enumerate(protocol_metrics):
        y_pos = len(protocol_metrics) - i - 0.5
        
        if metric not in df.columns:
            continue
        
        # Calculate statistics for each combination
        combo_stats = {}
        valid_combos = []
        
        for combo_name, combo_data in combination_data.items():
            stats = calculate_stats(combo_data, metric)
            if not np.isnan(stats['mean']) and stats['n'] > 0:
                combo_stats[combo_name] = stats
                valid_combos.append((combo_name, stats['mean']))
        
        # Determine best performer
        if valid_combos:
            if better_direction == 'lower':
                best_combo = min(valid_combos, key=lambda x: x[1])[0]
            else:
                best_combo = max(valid_combos, key=lambda x: x[1])[0]
        else:
            best_combo = 'N/A'
        
        # Generate insights focused on protocol behavior
        insights = {
            'Connection time (protocol)': 'Protocol efficiency',
            'Latency': 'Real-time capability',
            'Security handshake time': 'Security overhead',
            'Retransmissions': 'Protocol reliability',
            'RRC state changes': 'Connection management',
            'Network registration changes': 'Protocol stability',
        }
        
        # Draw row
        colors = ['white', 'lightgray']
        row_color = colors[i % 2]
        
        # Prepare data values for each combination
        combo_values = []
        combo_names = ['MQTT+NB-IoT', 'MQTT+LTE-M', 'LWM2M+NB-IoT', 'LWM2M+LTE-M']
        
        for combo_name in combo_names:
            if combo_name in combo_stats:
                stats = combo_stats[combo_name]
                if metric in ['Connection time (protocol)', 'Security handshake time', 'Latency']:
                    # Use 3 decimal places for time metrics
                    combo_values.append(f"{stats['mean']:.3f}±{stats['std']:.3f}\n(n={stats['n']})")
                else:
                    # Use 2 decimal places for other metrics
                    combo_values.append(f"{stats['mean']:.2f}±{stats['std']:.2f}\n(n={stats['n']})")
            else:
                combo_values.append('N/A')
        
        row_data = [f"{metric}\n[{unit}]"] + combo_values + [insights.get(metric, 'Protocol performance')]
        
        for j, (data_item, x_pos, width) in enumerate(zip(row_data, x_positions, col_widths)):
            # Color coding
            if j == 0:  # Metric column
                cell_color = row_color
            elif 1 <= j <= 4:  # Combination columns
                combo_name = combo_names[j-1]
                # Use the same color as in the header for highlighting
                if combo_name == best_combo:
                    cell_color = header_colors[j]  # Use header color for this column
                else:
                    cell_color = row_color
            else:  # Insight column
                cell_color = row_color

            rect = Rectangle((x_pos, y_pos - 0.35), width, 0.7,
                            facecolor=cell_color, alpha=0.7, edgecolor='black', linewidth=0.5)
            ax.add_patch(rect)

            # Font sizing
            font_size = 10 if len(str(data_item)) > 35 else 11

            # Text placement
            ax.text(x_pos + width/2, y_pos, str(data_item), ha='center', va='center',
                    fontsize=30, wrap=True, fontweight='bold')
    
    # Layout adjustments
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.02, left=0.01, right=0.99, top=0.96)
    plt.savefig(f'{save_path}/{file_name}.png', dpi=dpi, bbox_inches='tight', 
                facecolor='white', edgecolor='none', pad_inches=0.05)
    
    if show_plot:
        plt.show()
    else:
        plt.close()
    
    print(f"✅ Protocol comparison table saved to: {save_path}/{file_name}.png")
    print(f"📊 Focusing on {len(protocol_metrics)} protocol-relevant metrics only")

def create_interactive_map_only(df, save_path="tex_documentation/Figures", file_name="measurement_locations_interactive"):
    """
    Create ONLY an interactive HTML map showing all measurement locations
    
    Parameters:
    - df: DataFrame with your measurement data (must have 'lat' and 'lon' columns)
    - save_path: path to save the HTML file
    - file_name: name for the saved HTML file (without extension)
    """
    try:
        import folium
        
        # Get unique measurement locations (remove duplicates)
        locations = df[['lat', 'lon']].drop_duplicates().dropna()
        
        if len(locations) == 0:
            print("❌ No valid location data found")
            return
        
        print(f"📍 Found {len(locations)} unique measurement locations")
        
        # Calculate map bounds
        lat_min, lat_max = locations['lat'].min(), locations['lat'].max()
        lon_min, lon_max = locations['lon'].min(), locations['lon'].max()
        
        # Calculate center point
        center_lat = (lat_min + lat_max) / 2
        center_lon = (lon_min + lon_max) / 2
        
        print(f"🗺️  Map area: Lat [{lat_min:.6f}, {lat_max:.6f}], Lon [{lon_min:.6f}, {lon_max:.6f}]")
        print(f"📌 Center point: ({center_lat:.6f}, {center_lon:.6f})")
        
        # Create the interactive map with OpenStreetMap (has street names)
        m = folium.Map(
            location=[center_lat, center_lon],
            zoom_start=15,  # Good zoom level for street names
            tiles='OpenStreetMap'  # Clear street names and details
        )
        
        # Add measurement points with bigger, more visible markers
        for idx, row in locations.iterrows():
            folium.CircleMarker(
                location=[row['lat'], row['lon']],
                radius=5,  # Larger radius
                popup=f"<b>Measurement Point</b><br>Lat: {row['lat']:.6f}<br>Lon: {row['lon']:.6f}",
                color='red',
                fill=True,
                fillColor='red',
                fillOpacity=0.8,
                weight=3
            ).add_to(m)
        
        # Fit map to bounds with small padding
        padding = 0.005  # Small padding for tight view
        southwest = [lat_min - padding, lon_min - padding]
        northeast = [lat_max + padding, lon_max + padding]
        m.fit_bounds([southwest, northeast])
        
        # Add a nice title
        title_html = '''
                     <h2 align="center" style="font-size:24px; margin: 10px;">
                     <b>Measurement Locations</b><br>
                     <span style="font-size:16px; color: #666;">
                     </span>
                     </h2>
                     '''
        m.get_root().html.add_child(folium.Element(title_html))
        
        # Save as HTML
        html_path = f'{save_path}/{file_name}.html'
        m.save(html_path)
        print(f"✅ Interactive map saved to: {html_path}")
        print(f"🌐 Open this file in your browser to:")
        print(f"   - Zoom in/out for perfect detail level")
        print(f"   - See all street names clearly")
        print(f"   - Take a high-quality screenshot")
        print(f"   - Pan around to find the best view")
        
    except ImportError:
        print("❌ Folium not installed. Install with: pip install folium")

def plot_protocol_technology_kde_with_ci(data, data_name, title, unit="", file_name="", save_path="Figures/", show_plot=False, confidence_level=0.95, custom_x_limits=None, custom_y_limits=None):
    """
    Plot KDE distribution and bar chart with confidence intervals comparing all protocol-technology combinations
    
    Parameters:
    - data: DataFrame with the data
    - data_name: column name for the metric to plot (e.g., 'Latency')
    - title: title for the plots
    - unit: unit of measurement
    - file_name: name for saved file
    - save_path: path to save figures
    - show_plot: whether to display the plot
    - confidence_level: confidence level for CI (default 0.95)
    - custom_x_limits: tuple (x_min, x_max) for KDE plot X-axis limits
    - custom_y_limits: dict with 'kde': (y_min, y_max) and 'ci': (y_min, y_max) for Y-axis limits
    """
    import scipy.stats as stats
    
    # Create figure with 2 subplots (top: KDE, bottom: CI)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12))
    
    # Define the 4 combinations
    combinations = [
        ('MQTT', 'NB-IoT'),
        ('MQTT', 'LTE-M'),
        ('LWM2M', 'NB-IoT'),
        ('LWM2M', 'LTE-M')
    ]
    
    # Define colors for each combination
    colors = ['#E74C3C', '#3498DB', '#F39C12', '#2ECC71']  # Red, Blue, Orange, Green
    
    # =====================
    # TOP PLOT: KDE
    # =====================
    
    # Plot KDE for each combination
    for i, (protocol, tech) in enumerate(combinations):
        # Filter data for this specific combination
        filtered_data = data[
            (data['Transport protocol'] == protocol) & 
            (data['Communication technology'] == tech)
        ][data_name].dropna()
        
        if len(filtered_data) > 0:
            # Create label for this combination
            label = f'{protocol} + {tech} (n={len(filtered_data)})'
            
            # Plot KDE
            sns.kdeplot(data=filtered_data, ax=ax1, color=colors[i], 
                       linewidth=3, alpha=0.3, label=label, fill=True)
            
            # Calculate and add mean line
            mean_val = filtered_data.mean()
            ax1.axvline(mean_val, color=colors[i], linestyle='--', linewidth=2, 
                       alpha=0.9, label=f'{protocol}+{tech} Mean: {mean_val:.3f}')
        else:
            print(f"⚠️ No data found for {protocol} + {tech}")
    
    # Set CUSTOM X-axis limits for KDE
    if custom_x_limits is not None:
        ax1.set_xlim(custom_x_limits[0], custom_x_limits[1])
        print(f"✅ Using custom X-axis limits for KDE: [{custom_x_limits[0]}, {custom_x_limits[1]}]")
    
    # Set CUSTOM Y-axis limits for KDE
    if custom_y_limits is not None and 'kde' in custom_y_limits:
        ax1.set_ylim(custom_y_limits['kde'][0], custom_y_limits['kde'][1])
        print(f"✅ Using custom Y-axis limits for KDE: [{custom_y_limits['kde'][0]}, {custom_y_limits['kde'][1]}]")
    else:
        # Default Y-axis behavior
        y_min, y_max = ax1.get_ylim()
        y_padding = y_max * 0.1
        ax1.set_ylim(0, y_max + y_padding)
    
    # Customize KDE plot
    ax1.set_title(f"Kernel Density Estimation for {title}\nby Protocol and Technology", 
                 fontsize=20, fontweight='bold')
    ax1.set_xlabel(f"{title} [{unit}]", fontsize=18, fontweight='bold')

    if unit:
        ax1.set_ylabel(f'Probability Density [1/{unit}]', fontsize=18, fontweight='bold')
    else:
        ax1.set_ylabel('Probability Density', fontsize=18, fontweight='bold')

    ax1.grid(True, linestyle='--', alpha=0.8)
    ax1.legend(fontsize=12, loc='best', prop={'weight': 'bold'})
    ax1.tick_params(labelsize=12)
    
    # =====================
    # BOTTOM PLOT: CI
    # =====================
    
    # Calculate statistics for each combination
    stats_summary = {}
    means = []
    errors = []
    labels = []
    bar_colors = []
    ns = []
    
    for i, (protocol, tech) in enumerate(combinations):
        # Filter data for this specific combination
        filtered_data = data[
            (data['Transport protocol'] == protocol) & 
            (data['Communication technology'] == tech)
        ][data_name].dropna()
        
        if len(filtered_data) > 1:
            mean_val = filtered_data.mean()
            std_val = filtered_data.std()
            n = len(filtered_data)
            
            # Calculate confidence interval
            alpha = 1 - confidence_level
            t_critical = stats.t.ppf(1 - alpha/2, df=n-1)
            margin_error = t_critical * (std_val / np.sqrt(n))
            ci_lower = mean_val - margin_error
            ci_upper = mean_val + margin_error
            
            stats_summary[f'{protocol}+{tech}'] = {
                'mean': mean_val, 'std': std_val, 'n': n,
                'ci_lower': ci_lower, 'ci_upper': ci_upper,
                'margin_error': margin_error
            }
            
            means.append(mean_val)
            errors.append(margin_error)
            labels.append(f'{protocol}\n+\n{tech}')
            bar_colors.append(colors[i])
            ns.append(n)
            
        elif len(filtered_data) == 1:
            # Single data point - no CI possible
            mean_val = filtered_data.iloc[0]
            means.append(mean_val)
            errors.append(0)
            labels.append(f'{protocol}\n+\n{tech}')
            bar_colors.append(colors[i])
            ns.append(1)
            print(f"⚠️ Only 1 data point for {protocol} + {tech}, no CI calculated")
        else:
            print(f"⚠️ No data found for {protocol} + {tech}")
    
    if len(means) == 0:
        print("❌ No data available for any combination")
        return
    
    # Create bar chart
    x_pos = np.arange(len(labels))
    bars = ax2.bar(x_pos, means, yerr=errors, capsize=10, 
                  color=bar_colors, alpha=0.8, 
                  edgecolor='black', linewidth=1.5)
    
    # Calculate label positions
    label_heights = []
    for i, (bar, mean_val, error, n) in enumerate(zip(bars, means, errors, ns)):
        label_y = bar.get_height() + error + abs(bar.get_height()) * 0.05
        label_heights.append(label_y)
        ax2.text(bar.get_x() + bar.get_width()/2, label_y,
                f'{mean_val:.3f} ± {error:.3f}\n(n={n})',
                ha='center', va='bottom', fontsize=16, fontweight='bold')
    
    # Perform statistical tests if we have multiple groups
    if len(means) > 1:
        # Get all data for Kruskal-Wallis test (non-parametric ANOVA)
        all_groups = []
        group_names = []
        for protocol, tech in combinations:
            filtered_data = data[
                (data['Transport protocol'] == protocol) & 
                (data['Communication technology'] == tech)
            ][data_name].dropna()
            
            if len(filtered_data) > 0:
                all_groups.append(filtered_data.values)
                group_names.append(f'{protocol}+{tech}')
        
        if len(all_groups) > 1:
            try:
                from scipy.stats import kruskal
                statistic, p_value = kruskal(*all_groups)
                significance = "***" if p_value < 0.001 else "**" if p_value < 0.01 else "*" if p_value < 0.05 else "ns"
                
                ax2.text(0.5, 0.95, f'Kruskal-Wallis: p = {p_value:.4f} {significance}', 
                        transform=ax2.transAxes, ha='center', va='top', fontsize=16,
                        bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
            except:
                print("⚠️ Could not perform statistical test")
    
    # Set CUSTOM Y-axis limits for CI plot
    if custom_y_limits is not None and 'ci' in custom_y_limits:
        ax2.set_ylim(custom_y_limits['ci'][0], custom_y_limits['ci'][1])
        print(f"✅ Using custom Y-axis limits for CI: [{custom_y_limits['ci'][0]}, {custom_y_limits['ci'][1]}]")
    else:
        # Default Y-axis behavior
        if label_heights:
            bar_bottoms = [mean - error for mean, error in zip(means, errors)]
            y_min_bar = min(bar_bottoms)
            y_max_bar = max(label_heights)
            
            y_range_bar = y_max_bar - y_min_bar
            y_padding_bottom = y_range_bar * 0.1 if y_min_bar < 0 else 0
            y_padding_top = y_range_bar * 0.1
            
            ax2.set_ylim(y_min_bar - y_padding_bottom, y_max_bar + y_padding_top)
    
    # Customize CI plot
    ax2.set_title(f"{title}: Comparison by Protocol and Technology\nwith {confidence_level*100:.0f}% Confidence Intervals", 
                 fontsize=20, fontweight='bold')
    ax2.set_ylabel(f"{title} [{unit}]", fontsize=18, fontweight='bold')
    ax2.set_xlabel("Protocol + Communication Technology", fontsize=18, fontweight='bold')
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels(labels, fontweight='bold', fontsize=16)
    ax2.grid(True, linestyle='--', alpha=0.8, axis='y')
    ax2.tick_params(labelsize=16)

    # Adjust layout
    plt.tight_layout(pad=3.0)
    plt.savefig(f'{save_path}/{file_name}_protocol_tech_kde_ci.png', dpi=300, 
                bbox_inches='tight', pad_inches=0.3,
                facecolor='white', edgecolor='none')
    
    if show_plot:
        plt.show()
    else:
        plt.close()
        
    print(f"✅ Combined KDE+CI plot saved to: {save_path}/{file_name}_protocol_tech_kde_ci.png")


if __name__ == "__main__":
    data = get_data()
    df = create_dataframe(data)

    # Create radar chart
    create_radar_chart(df, metrics=None, group_by='Communication technology', save_path='tex_documentation/Figures', show_plot=False)

    # Plot statistics for Connection time (link), Tx Power, RSRQ, RSRP, SNR, and Downlink pathloss, link uplink frequency, link downlink frequency, tx repetitions, rx repetitions, energy estimate, throughput, downlink pathloss,
    plot_statistics_kde_with_ci(df, data_name='TX power', filter_name='Communication technology',
                                title='TX Power', unit='dBm', file_name='tx_power',
                                save_path='tex_documentation/Figures', show_plot=False, confidence_level=0.95, custom_x_limits=(-40, 35),  # X-axis from -30 to 25 dBm
                                custom_y_limits={'kde': (0, 0.02), 'bar': (-1, 9)})
    plot_statistics_kde_with_ci(df, data_name='Connection time (link)', filter_name='Communication technology',
                                title='Connection Time (Link)', unit='s', file_name='connection_time_link',
                                save_path='tex_documentation/Figures', show_plot=False, confidence_level=0.95, custom_x_limits=(-15, 100),  # X-axis from 0 to 200 seconds
                                custom_y_limits={'kde': (0, 0.09), 'bar': (-1, 15)})
    plot_statistics_kde_with_ci(df, data_name='RSRQ', filter_name='Communication technology',
                                title='RSRQ', unit='dB', file_name='rsrq',
                                save_path='tex_documentation/Figures', show_plot=False, confidence_level=0.95, custom_x_limits=(-25, 0),  # X-axis from -30 to 0 dB
                                custom_y_limits={'kde': (0, 0.1), 'bar': (-15, 5)})
    plot_statistics_kde_with_ci(df, data_name='RSRP', filter_name='Communication technology',
                                title='RSRP', unit='dBm', file_name='rsrp',
                                save_path='tex_documentation/Figures', show_plot=False, confidence_level=0.95, custom_x_limits=(-140, -30),  # X-axis from -140 to -30 dBm
                                custom_y_limits={'kde': (0, 0.02), 'bar': (-100, 20)})
    plot_statistics_kde_with_ci(df, data_name='SNR', filter_name='Communication technology',
                                title='SNR', unit='dB', file_name='snr',
                                save_path='tex_documentation/Figures', show_plot=False, confidence_level=0.95, custom_x_limits=(-20, 35),  # X-axis from -30 to 0 dB
                                custom_y_limits={'kde': (0, 0.03), 'bar': (-1, 15)})
    plot_statistics_kde_with_ci(df, data_name='Downlink pathloss', filter_name='Communication technology',
                                title='Downlink Pathloss', unit='dB', file_name='downlink_pathloss',
                                save_path='tex_documentation/Figures', show_plot=False, confidence_level=0.95, custom_x_limits=(0, 170),  # X-axis from 10 to 150 dB
                                custom_y_limits={'kde': (0, 0.02), 'bar': (-10, 140)})
    plot_statistics_kde_with_ci(df, data_name='TX repetitions', filter_name='Communication technology',
                                title='TX Repetitions', unit='count', file_name='tx_repetitions',
                                save_path='tex_documentation/Figures', show_plot=False, confidence_level=0.95, custom_x_limits=(-1, 5),  # X-axis from 0 to 20 repetitions
                                custom_y_limits={'kde': (0, 1.5), 'bar': (-0.2, 1.5)})
    plot_statistics_kde_with_ci(df, data_name='RX repetitions', filter_name='Communication technology',
                                title='RX Repetitions', unit='count', file_name='rx_repetitions',
                                save_path='tex_documentation/Figures', show_plot=False, confidence_level=0.95, custom_x_limits=(-2, 20),  # X-axis from 0 to 20 repetitions
                                custom_y_limits={'kde': (0, 0.35), 'bar': (-0.1, 3)})
    plot_statistics_kde_with_ci(df, data_name='Energy estimate', filter_name='Communication technology',
                                title='Energy Estimate', unit='relative', file_name='energy_estimate',
                                save_path='tex_documentation/Figures', show_plot=False, confidence_level=0.95, custom_x_limits=(3, 11),  # X-axis from 0 to 10 Joules
                                custom_y_limits={'kde': (0, 0.3), 'bar': (-0.1, 10)})
    plot_statistics_kde_with_ci(df, data_name='Throughput (link)', filter_name='Communication technology',
                                title='Throughput (Link)', unit='kbps', file_name='throughput_link',
                                save_path='tex_documentation/Figures', show_plot=False, confidence_level=0.95, custom_x_limits=(-20, 500),  # X-axis from 0 to 600 kbps
                                custom_y_limits={'kde': (0, 0.0125), 'bar': (-1, 300)})
    # Replace these lines in your main section:

    # Use the new frequency-specific function
    plot_frequency_kde(df, data_name='Uplink frequency', filter_name='Communication technology',
                    title='Uplink Frequency', unit='MHz', file_name='uplink_frequency',
                    save_path='tex_documentation/Figures', show_plot=False, 
                    custom_x_limits=(650, 2000),
                    custom_y_limits=(0, 0.025))

    plot_frequency_kde(df, data_name='Downlink frequency', filter_name='Communication technology',
                    title='Downlink Frequency', unit='MHz', file_name='downlink_frequency',
                    save_path='tex_documentation/Figures', show_plot=False, 
                    custom_x_limits=(500, 2000),
                    custom_y_limits=(0, 0.01))
    

    # With single combined calls:
    plot_protocol_technology_kde_with_ci(df, data_name='Latency', 
                                        title='Latency', unit='s', file_name='latency_all',
                                        save_path='tex_documentation/Figures', show_plot=False, 
                                        confidence_level=0.95,
                                        custom_x_limits=(-0.5, 3),
                                        custom_y_limits={'kde': (0, 6), 'ci': (-0.1, 2)})

    plot_protocol_technology_kde_with_ci(df, data_name='Security handshake time', 
                                        title='Security Handshake Time', unit='s', file_name='handshake_time_all',
                                        save_path='tex_documentation/Figures', show_plot=False, 
                                        confidence_level=0.95,
                                        custom_x_limits=(-2, 20),
                                        custom_y_limits={'kde': (0, 0.5), 'ci': (-0.5, 12)})

    plot_protocol_technology_kde_with_ci(df, data_name='Retransmissions', 
                                        title='Retransmissions', unit='count', file_name='retransmissions_all',
                                        save_path='tex_documentation/Figures', show_plot=False, 
                                        confidence_level=0.95,
                                        custom_x_limits=(-1, 18),
                                        custom_y_limits={'kde': (0, 1), 'ci': (-0.5, 15)})

    plot_protocol_technology_kde_with_ci(df, data_name='RRC state changes', 
                                        title='RRC State Changes', unit='count', file_name='rrc_state_changes_all',
                                        save_path='tex_documentation/Figures', show_plot=False, 
                                        confidence_level=0.95,
                                        custom_x_limits=(1, 12),
                                        custom_y_limits={'kde': (0, 2.5), 'ci': (-0.5, 8)})

    plot_protocol_technology_kde_with_ci(df, data_name='Network registration changes', 
                                        title='Network Registration Changes', unit='count', file_name='network_registration_changes_all',
                                        save_path='tex_documentation/Figures', show_plot=False, 
                                        confidence_level=0.95,
                                        custom_x_limits=(5, 20),
                                        custom_y_limits={'kde': (0, 3.5), 'ci': (-0.5, 20)})

    plot_protocol_technology_kde_with_ci(df, data_name='Connection time (protocol)', 
                                        title='Connection Time (Protocol)', unit='s', file_name='connection_time_all',
                                        save_path='tex_documentation/Figures', show_plot=False, 
                                        confidence_level=0.95,
                                        custom_x_limits=(-10, 20),
                                        custom_y_limits={'kde': (0, 0.4), 'ci': (-0.5, 10)})

    
    # Create comparison table
    create_communication_technology_table(df, save_path="tex_documentation/Figures", 
                                         file_name="communication_technology_comparison", 
                                         show_plot=False)
    
    # Replace the protocol comparison with 4-way comparison
    create_4way_comparison_table(df, save_path="tex_documentation/Figures", 
                                file_name="4way_protocol_technology_comparison", 
                                show_plot=False)
    
    # Create measurement locations map
    create_interactive_map_only(df)
