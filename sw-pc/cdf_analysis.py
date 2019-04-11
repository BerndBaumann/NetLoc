#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
#import networkx as nx
import collections
import sys
from scipy.optimize import minimize


class LogfileAnalyzer(object):
    def __init__(self, filename, reference_positions, sink_addr='', xref_addr='', yref_addr=''):
        self.fn = filename

        # list of lists of addresses [['a523', '9d23', ...], ['a523', '4010', ...], ...]
        # each list provides the corresponding indices for the self.distances list
        self.addresses = []
        self.sink_addr = sink_addr
        self.xref_addr = xref_addr
        self.yref_addr = yref_addr

        # list of unique addresses in the network
        self.network_addresses = list(reference_positions.keys())

        # list of dinstace matrices [mat0, mat1, mat2, ...]
        self.distances = []

        # matrix of counters. Column and row indices
        # are the same as for self.addresses and matrix
        # elements are the number of elements that are != zero
        self.cnt_distances = np.zeros(1)

        # mean of all distance measurements between any two nodes
        self.avg_distances = np.zeros(1)

        # variance of all distance measurements between any two nodes
        self.var_distances = np.zeros(1)

        # keys are the addresses from self.addresses,
        # values are the node (world-)coordinates
        self.reference_positions = reference_positions

        # distances, calculated based on the reference positions
        self.reference_distances = np.zeros(1)

        # List of dictionaries. List item x corresponds to the
        # positions calculated from item x in self.distances
        self.positions = []

        # mean position of all measurements of one node
        self.avg_positions = {}

        # dictionary of lists {'addr' : [err1, err2, ...]} of all
        # position errors for each node
        self.position_errors = {}

        # dictionary mapping addresses to correction distances
        self.distance_correction = {}

        self._calc_reference_distances()

        self.last_position_as_initial_position = False

    def readfile_to_matrix(self, correction=False, verbose=True):
        self.positions.clear()
        self.position_errors.clear()
        self.cnt_distances = np.zeros(1)
        self.var_distances = np.zeros(1)
        self.avg_distances = np.zeros(1)
        self.addresses.clear()
        self.distances.clear()

        node_statistics = {}
        for sender in self.network_addresses:
            node_statistics[sender] = dict(zip(self.network_addresses, [dict()] * len(self.network_addresses)))
            for neighbor in node_statistics[sender].keys():
                node_statistics[sender][neighbor] = {'dist': 0.0, 'count': 0.0}

        with open(self.fn, 'r') as fp:
            current_seqnum = 42
            current_matrix = []
            current_addresses = []
            skipped_counter = 0
            line = fp.readline()
            while line:
                line = fp.readline()
                if line.startswith("ERROR: UDP timeout"):
                    skip_measurement_number = int(line.split('sn=')[1].split(')')[0])
                if line.startswith("RPLUWB"):
                    measurement = line.strip().split(',')
                    sender = measurement[1].strip()
                    if sender not in self.network_addresses:
                        continue
                    seqnum = int(measurement[2])
                    new_edges = []  # list of tuples [(nodeA, nodeB, distance), ...]
                    for new_edge in list(zip(measurement[3::2], measurement[4::2])):
                        neighbor = new_edge[0].strip()
                        distance = float(new_edge[1])
                        if neighbor != '0000' and distance > 0.0 and neighbor in self.network_addresses:
                            if correction:
                                new_edges.append((sender, neighbor, distance + self.distance_correction[sender]))
                            else:
                                new_edges.append((sender, neighbor, distance))
                                node_statistics[sender][neighbor]['dist'] += distance
                                node_statistics[sender][neighbor]['count'] += 1
                    if not new_edges:
                        # skip empty measurements but update sequence number
                        current_seqnum = seqnum
                        skipped_counter += 1
                        continue

                    if seqnum != current_seqnum:
                        if len(current_matrix) > 1 and current_seqnum != skip_measurement_number:
                            self.distances.append(current_matrix)
                            self.addresses.append(current_addresses)
                            if verbose:
                                print("\n--- {} ---\n".format(current_seqnum), '\n'.join([str(x) for x in current_matrix]))
                        current_matrix = []
                        current_addresses = []

                    # Add measurement to current distance matrix
                    for new_edge in new_edges:
                        if not new_edge[0] in current_addresses:
                            current_addresses.append(new_edge[0])
                            # add a new column
                            for row in current_matrix:
                                row.append(0.0)
                            # add a new row
                            current_matrix.append([0.0] * len(current_addresses))
                        if not new_edge[1] in current_addresses:
                            current_addresses.append(new_edge[1])
                            # add a new column
                            for row in current_matrix:
                                row.append(0.0)
                            # add a new row
                            current_matrix.append([0.0] * len(current_addresses))
                        row_index = current_addresses.index(new_edge[0])
                        col_index = current_addresses.index(new_edge[1])
                        if current_matrix[row_index][col_index] != 0.0:
                            current_matrix[row_index][col_index] = (new_edge[2] + current_matrix[row_index][
                                col_index]) / 2.0
                        else:
                            current_matrix[row_index][col_index] = new_edge[2]
                        current_matrix[col_index][row_index] = new_edge[2]

                    current_seqnum = seqnum
        if not correction:
            for sender, neighbors in node_statistics.items():
                if verbose:
                    print("Analyzing node", sender)
                err_sum = 0.0
                err_count = 0.0
                for neighbor, stats in neighbors.items():
                    try:
                        row = self.network_addresses.index(sender)
                        col = self.network_addresses.index(neighbor)
                        real_dist = self.reference_distances[row][col]
                        meas_dist = stats['dist'] / stats['count']
                        err = real_dist - meas_dist
                        err_sum += err
                        err_count += 1.0
                        if verbose:
                            print("\t[{}] | dist = {: >8.3f} m; count = {: >5}; real_dist = {: >8.3f} m; err = {: "
                                  ">8.3f} cm".format(neighbor,
                                                    meas_dist,
                                                    stats['count'],
                                                    real_dist,
                                                    err))
                    except ZeroDivisionError:
                        pass
                if verbose:
                    print("\tmean err of {} is {} m".format(sender, err_sum / err_count))
                self.distance_correction[sender] = err_sum / err_count
            if verbose:
                print("Distance corrections:")
                for k, v in self.distance_correction.items():
                    print("{} : {: >8.3f}".format(k, v))

    def __len__(self):
        return len(self.distances)

    def __str__(self):
        ret = "File: {}\n".format(self.fn)
        ret += "Number of measurements: {}\n".format(len(self))
        ret += "Number of nodes: {}\n".format(len(self.network_addresses))
        ret += "Node addresses: {}\n".format(self.network_addresses)
        return ret

    def save_to_file(self, filename=None):
        import pickle
        if filename is None:
            filename = self.fn + ".bin"
        with open(filename, "wb") as outfile:
            pickle.dump(self, outfile, pickle.HIGHEST_PROTOCOL)
        return filename

    def set_addresses(self, sink_addr, xref, yref):
        self.sink_addr = sink_addr
        self.xref_addr = xref
        self.yref_addr = yref

    def get_average_distances(self):
        if len(self.distances) == 0:
            self.readfile_to_matrix()

        self.avg_distances = np.zeros((len(self.network_addresses), len(self.network_addresses)))
        self.cnt_distances = np.zeros((len(self.network_addresses), len(self.network_addresses)))
        # Matrices in self.distances can have different sizes! Their measurements can be linked
        # neighbors using the self.addresses list
        for idx in range(len(self.distances)):
            mat = self.distances[idx]
            addr = self.addresses[idx]
            for row in range(0, len(addr)):
                for col in range(row + 1, len(addr)):
                    if mat[row][col] > 0.0:
                        # find index in summarizing matrix
                        row_ = self.network_addresses.index(addr[row])
                        col_ = self.network_addresses.index(addr[col])
                        self.avg_distances[row_][col_] += mat[row][col]
                        self.avg_distances[col_][row_] = self.avg_distances[row_][col_]
                        self.cnt_distances[row_][col_] += 1
                        self.cnt_distances[col_][row_] = self.cnt_distances[row_][col_]

        for row in range(0, len(self.network_addresses)):
            for col in range(row + 1, len(self.network_addresses)):
                if self.cnt_distances[row][col] > 0:
                    self.avg_distances[row][col] /= self.cnt_distances[row][col]
                    self.avg_distances[col][row] = self.avg_distances[row][col]
        return self.avg_distances

    def get_var_distances(self):
        if len(self.avg_distances) == 1:
            self.get_average_distances()

        self.var_distances = np.zeros((len(self.network_addresses), len(self.network_addresses)))
        for idx in range(len(self.distances)):
            mat = self.distances[idx]
            addr = self.addresses[idx]
            for row in range(0, len(addr)):
                for col in range(row + 1, len(addr)):
                    if mat[row][col] > 0.0:
                        # find index in summarizing matrix
                        row_ = self.network_addresses.index(addr[row])
                        col_ = self.network_addresses.index(addr[col])
                        self.var_distances[row_][col_] += np.square(mat[row][col] - self.avg_distances[row_][col_])
        for row in range(0, len(self.network_addresses)):
            for col in range(row + 1, len(self.network_addresses)):
                if self.cnt_distances[row][col] > 0:
                    self.var_distances[row][col] /= float(self.cnt_distances[row][col])
                    self.var_distances[col][row] = self.var_distances[row][col]
        return self.var_distances

    def get_distances_single_node(self, from_addr, to_addr):
        if not (from_addr in self.network_addresses and to_addr in self.network_addresses):
            print("Address not found in ", self.network_addresses)
            return None
        res = []
        for idx in range(len(self.distances)):
            mat = self.distances[idx]
            addr = self.addresses[idx]
            try:
                if mat[addr.index(from_addr)][addr.index(to_addr)] > 0.0:
                    res.append(mat[addr.index(from_addr)][addr.index(to_addr)])
            except ValueError:
                pass
        return res

    def _calc_reference_distances(self):
        num_addr = len(self.network_addresses)
        self.reference_distances = np.zeros((num_addr, num_addr))
        for row in range(0, num_addr):
            node_from = self.reference_positions[self.network_addresses[row]]
            for col in range(row + 1, len(self.network_addresses)):
                node_to = self.reference_positions[self.network_addresses[col]]
                self.reference_distances[row][col] = np.linalg.norm(np.array(node_from) - np.array(node_to))
                self.reference_distances[col][row] = self.reference_distances[row][col]
        # self.reference_distances /= 100.0

    def get_avg_distance_error(self):
        if len(self.reference_positions) == 0:
            print("Set reference positions first!")
            return None

        if len(self.avg_distances) == 1:
            self.get_average_distances()

        avg_dist_error = np.zeros((len(self.network_addresses), len(self.network_addresses)))
        for row in range(0, len(self.network_addresses)):
            for col in range(row + 1, len(self.network_addresses)):
                if self.avg_distances[row][col] > 0.0:
                    avg_dist_error[row][col] = self.reference_distances[row][col] - self.avg_distances[row][col]
                    avg_dist_error[col][row] = avg_dist_error[row][col]

        # calculate average distance error
        for a in self.network_addresses:
            s = 0
            count = 0
            row = self.network_addresses.index(a)
            for col in range(len(self.network_addresses)):
                if self.avg_distances[row][col] > 0.0:
                    count += 1
                    s += avg_dist_error[row][col]
            self.distance_correction[a] = s / count
        return avg_dist_error

    def _calc_positions(self):
        if self.sink_addr == '' or self.xref_addr == '' or self.yref_addr == '':
            print("Set addresses for sink, x-reference and y-reference first")
            return

        if len(self.reference_distances) == 0:
            print("Set reference positions first!")
            return

        self.positions.clear()
        sink_pos = np.matrix([[float(self.reference_positions[self.sink_addr][0])],
                              [float(self.reference_positions[self.sink_addr][1])]])
        map_rotation = 0.0  # np.arctan2(coordinates['9d23'][1], coordinates['9d23'][0])

        tmpcnt = 0
        for idx in range(len(self.distances)):
            mat = self.distances[idx]
            addr = self.addresses[idx]

            # Calculate the distances when projected onto the (x,y) plane
            projected_distances, projected_refs = self._project_distances(np.array(mat), addr)
            if self.last_position_as_initial_position and len(self.positions) > 0:
                coordinates = self._localize(projected_distances, addr, self.positions[-1])
            else:
                coordinates = self._localize(projected_distances, addr)
            # Get the positions of the nodes using the Spring-Layout graph plotting function
            # np_mat = np.array(projected_distances, dtype=[('len', np.float)])
            # graph = nx.from_numpy_array(np_mat)
            # graph = nx.relabel_nodes(graph, dict(zip(range(len(addr)), addr)))
            # coordinates = nx.drawing.nx_agraph.pygraphviz_layout(
            #     graph,
            #     prog='neato',
            #     args='-Gmaxiter=1000 -Gstart=432 -Gepsilon=1e-5 -Gratio=1.0')
            # # transform to the vector space with sink in origin
            # coordinates = self._transform_result(
            #     coordinates,
            #     self.sink_addr,
            #     x_node=self.xref_addr,
            #     y_node=self.yref_addr,
            #     ref=projected_refs)
            # # transform to map view
            # coordinates = self._transform_to_map_view(coordinates, sink_pos, map_rotation)
            self.positions.append(coordinates)
            tmpcnt += 1
            if tmpcnt % 100 == 0:
                sys.stdout.write("\rprogress: {:5.2f}%".format(100.0 * float(tmpcnt)/float(len(self.distances))))

    def _project_distances(self, distance_matrix, addresses):
        # projected distance = sqrt( (measured distance)^2 - z_diff^2 )
        projected = np.zeros((len(distance_matrix), len(distance_matrix)))
        for row in range(0, len(distance_matrix)):
            for col in range(row + 1, len(distance_matrix)):
                if distance_matrix[row, col] == 0.0:
                    continue
                z_diff = self.reference_positions[addresses[row]][2] - self.reference_positions[addresses[col]][2]
                # z_diff /= 100.0
                projected[row, col] = np.sqrt(np.square(distance_matrix[row, col]) - np.square(z_diff))
                projected[col, row] = projected[row, col]
        projected_r = dict()
        for k, v in self.reference_positions.items():
            projected_r[k] = (v[0], v[1])
        return projected, projected_r

    def _transform_result(self, coordinates, origin_key, x_node=None, y_node=None, ref=None):
        dx = coordinates[origin_key][0]
        dy = coordinates[origin_key][1]
        transformed = {}

        # displacement
        for k, v in coordinates.items():
            transformed[k] = ((v[0] - dx), (v[1] - dy))

        if x_node is None or x_node not in list(transformed.keys()):
            return transformed

        # scale
        if ref is None:
            scale = 1.4
        else:
            wrong_distance = np.linalg.norm(np.array(coordinates[x_node]) - np.array(coordinates[origin_key]))
            if type(ref) is float:
                scale = ref / wrong_distance
            elif type(ref) is dict:
                real_distance = np.linalg.norm(np.array(ref[x_node]) - np.array(ref[origin_key]))
                scale = real_distance / wrong_distance
        for k, v in transformed.items():
            transformed[k] = (v[0] * scale, v[1] * scale)

        # + rotation
        # y_node_x = transformed[y_node][0]
        # y_node_y = transformed[y_node][1]
        x_node_x = transformed[x_node][0]
        x_node_y = transformed[x_node][1]
        phi = np.arctan2(x_node_y, x_node_x)
        rot = np.matrix([[np.cos(phi), np.sin(phi)],
                         [-np.sin(phi), np.cos(phi)]])
        rotated = {}
        for k, v in transformed.items():
            tmp = rot * np.matrix(transformed[k]).T
            rotated[k] = (float(tmp[0]), float(tmp[1]))

        if y_node is None or y_node not in list(transformed.keys()):
            return rotated

        # + mirroring
        # m = np.array([[dx, x_node_x, y_node_x],
        #               [dy, x_node_y, y_node_y],
        #               [1, 1, 1]])
        # if np.linalg.det(m) < 0:
        if float(rotated[y_node][1]) < 0.0:
            mirrored = {}
            for k, v in rotated.items():
                mirrored[k] = (v[0], -v[1])
            return mirrored
        return rotated

    def _transform_to_map_view(self, coordinates, displacement, angle):
        transformed = {}
        for k, v in coordinates.items():
            rot = np.matrix([[np.cos(angle), np.sin(angle)],
                             [-np.sin(angle), np.cos(angle)]])
            tmp = rot * np.matrix(coordinates[k]).T + displacement
            transformed[k] = (float(tmp[0]), float(tmp[1]))
        return transformed

    def get_average_position(self):
        if len(self.positions) == 0:
            self._calc_positions()

        # initialize self.avg_positions with (0.0, 0.0) coordinates
        if len(self.network_addresses) != len(self.avg_positions.keys()):
            self.avg_positions = dict(zip(self.network_addresses, [(0.0, 0.0)]*len(self.network_addresses)))

        for pos in self.positions:
            for k, v in pos.items():
                self.avg_positions[k] = (self.avg_positions[k][0] + v[0], self.avg_positions[k][1] + v[1])
        for k, v in self.avg_positions.items():
            x_avg = self.avg_positions[k][0] / float(len(self.positions))
            y_avg = self.avg_positions[k][1] / float(len(self.positions))
            self.avg_positions[k] = (x_avg, y_avg)
        return self.avg_positions

    def get_position_errors(self):
        if len(self.reference_positions) == 0:
            print("Set reference positions")
            return None

        if len(self.positions) == 0:
            print("Calculating positions")
            self._calc_positions()

        self.position_errors.clear()
        ref_cm = {}
        for k, v in self.reference_positions.items():
            ref_cm[k] = np.array([v[0], v[1]])
            self.position_errors[k] = []
        for pos in self.positions:
            for k, v in pos.items():
                err = np.linalg.norm(ref_cm[k] - np.array(v))
                self.position_errors[k].append(err)
        return self.position_errors

    def plot_cdf(self, addr, show=False):
        if addr not in self.network_addresses:
            print("ERROR: Address not known: {}".format(addr))
            return

        if len(self.position_errors) == 0:
            print("Calculating position errors")
            self.get_position_errors()

        fig0 = plt.figure(figsize=(8, 6))
        ax0 = fig0.add_subplot(111)
        ax0.plot(np.array(sorted(self.position_errors[addr])),
                 np.linspace(0.0, 100.0, len(self.position_errors[addr])),
                 'bo--')
        ax0.grid(True)
        ax0.set_title("CDF of Node {}".format(addr))
        ax0.set_xlabel("error [cm]")
        ax0.set_ylabel("probability [%]")
        ax0.set_xlim([0, 100])
        ax0.set_ylim([0, 110])
        ax0.set_xticks(list(np.linspace(0, 100, 11)))
        ax0.set_yticks(list(np.linspace(0, 100, 11)))
        if show:
            fig0.show()
        else:
            fig0.savefig("cdf_{}.png".format(addr), dpi=300, format='png')

    def plot_total_cdf(self, exclude=[], show=False):
        all_errors = []
        for a in self.network_addresses:
            if a not in exclude:
                all_errors.extend(self.position_errors[a])
        print("Total number of position estimates:", len(all_errors))
        fig0 = plt.figure(figsize=(10,8))
        fig0.tight_layout()
        ax0 = fig0.add_subplot(111)
        ax0.plot(np.array(sorted(all_errors)),
                 np.linspace(0.0, 100.0, len(all_errors)),
                 'bo--')
        ax0.grid(True)
        # ax0.set_title("CDF of all Position Estimates")
        ax0.set_xlabel("error [cm]", fontsize=21)
        ax0.set_ylabel("probability [%]", fontsize=21)
        ax0.set_xlim([0, 100])
        ax0.set_ylim([0, 110])
        ax0.set_xticks(list(np.linspace(0, 100, 11)))
        ax0.set_yticks(list(np.linspace(0, 100, 11)))
        if show:
            fig0.show()
        else:
            fig0.savefig("cdf_total.png", dpi=300, format='png')

    def plot_cdf_and_error(self, addr, show=False):
        if addr not in self.network_addresses:
            print("ERROR: Address not known: {}".format(addr))
            return

        if len(self.position_errors) == 0:
            print("Calculating position errors")
            self.get_position_errors()

        fig0 = plt.figure(figsize=(10, 8))
        ax0 = fig0.add_subplot(211)
        ax0.plot(np.array(sorted(self.position_errors[addr])),
                 np.linspace(0.0, 100.0, len(self.position_errors[addr])),
                 'bo--')
        ax0.grid(True)
        ax0.set_title("CDF of Node {}".format(addr))
        ax0.set_xlabel("error [cm]")
        ax0.set_ylabel("probability [%]")
        ax0.set_xlim([0, 100])
        ax0.set_ylim([0, 110])
        ax0.set_xticks(list(np.linspace(0, 100, 11)))
        ax0.set_yticks(list(np.linspace(0, 100, 11)))

        ax1 = fig0.add_subplot(212)
        ax1.plot(np.array(self.position_errors[addr]), 'b-', label="Error of Node {}".format(addr))
        ax1.grid(True)
        ax1.legend()
        ax1.set_xlabel("Measurement Number")
        ax1.set_ylabel("error [cm]")
        ax1.set_ylim([0, 100])
        ax1.set_xlim([0, len(self.position_errors[addr])])
        ax0.set_xticks(list(np.linspace(0, 100, 11)))

        if show:
            fig0.show()
        else:
            fig0.savefig("cdf_{}_error.png".format(addr), dpi=300, format='png')

    def plot_all_positions(self, addresses=[], plot_real=False):
        for a in addresses:
            if a not in self.network_addresses:
                print("ERROR: address not known: ", a)
                return

        fig0 = plt.figure(figsize=(6, 10))
        fig0.tight_layout()
        ax0 = fig0.add_subplot(111)
        ax0.grid(True)
        ax0.set_xlim([-30, 660])
        ax0.set_ylim([-30, 1030])
        ax0.set_aspect('equal')
        ax0.set_xlabel('x [cm]', fontsize=21)
        ax0.set_ylabel('y [cm]', fontsize=21)
        ax0.set_title('Scatter Plot of all Positions', fontsize=21)
        # c = ['b', 'm', 'r', 'c', 'dark pink', 'k', 'y', 'g', 'orange']
        c = ['tab:blue', 'tab:pink', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:orange', 'tab:gray',
             'tab:olive', 'tab:cyan']
        colormap = dict(zip(addresses, c[:len(addresses)]))
        for a in addresses:
            x = []
            y = []
            for p in self.positions:
                try:
                    x.append(p[a][0])
                    y.append(p[a][1])
                except KeyError:
                    pass
            ax0.scatter(x, y, s=50, color=colormap[a], alpha=0.1)
            if plot_real:
                ax0.annotate(a,
                             xy=(self.reference_positions[a][0], self.reference_positions[a][1]),
                             xytext=(-20, 20),
                             textcoords='offset points',
                             ha='center',
                             va='center',
                             fontsize=12,
                             bbox=dict(boxstyle='round, pad=0.5', fc=colormap[a], alpha=0.5),
                             arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
                ax0.scatter(self.reference_positions[a][0], self.reference_positions[a][1], color='k', s=200,
                            alpha=1.0, marker='x')
        wall0 = 630.0
        wall1 = 1000.0
        room_corners = np.array([[0, 0], [wall0, 0], [wall0, wall1], [0, wall1], [0, 0]])
        ax0.plot(room_corners[:, 0], room_corners[:, 1], 'k-')
        fig0.savefig("all_positions.png", dpi=300, format='png')

    def _localize(self, distance_matrix, addresses, last={}, sink='a523', xnode='9d23', ynode='34c6'):
        ret = dict()
        try:
            smat_idx = addresses.index(sink)
            xmat_idx = addresses.index(xnode)
            ymat_idx = addresses.index(ynode)
        except ValueError:
            print("Cannot calculate coordinates. Initial nodes are missing")
            return ret
        # start with the triangle sink-xnode-ynode
        ret[sink] = np.array([self.reference_positions[sink][0],
                             self.reference_positions[sink][1]])
        ret[xnode] = np.array([ret[sink][0] + distance_matrix[smat_idx][xmat_idx],
                              ret[sink][1]])
        # localize ynode. must have a positive y coordinate
        initial_guess = np.array([(ret[xnode][0] - ret[sink][0]) / 2.0,
                                  ret[xnode][1] + 100.0])
        # error function for nodes with more than 2 neighbors
        def errfun(x, c, d):
            e = 0.0
            for k in range(len(c)):
                if d[k] != 0.0:
                    e += np.square(np.linalg.norm(x - c[k]) - d[k])
            return e
        # error function for nodes with 2 or 1 neighbors
        def errfun_penalty(x, c, d):
            e = 0.0
            mindist = max(d)
            for k in range(len(c)):
                if d[k] != 0.0:
                    e += np.square(np.linalg.norm(x - c[k]) - d[k])
                else:
                    dist2node = np.linalg.norm(x - c[k])
                    if dist2node < mindist:
                        e += np.square(dist2node - mindist)
            return e
        #
        coords = [ret[sink], ret[xnode]]
        distances = [distance_matrix[ymat_idx][smat_idx], distance_matrix[ymat_idx][xmat_idx]]
        r = minimize(errfun, initial_guess, args=(coords, distances), method='CG')
        # print(r)
        yy = 0.0
        # check if constraint for ynode holds, else mirror it
        if r.x[1] < ret[sink][1]:
            yy = ret[sink][1] + (ret[sink][1] - r.x[1])
        else:
            yy = r.x[1]
        ret[ynode] = np.array([r.x[0], yy])
        # list of nodes that are not located yet
        nodes_todo = [addr for addr in addresses if addr not in ret.keys()]
        while nodes_todo:
            # count number of neighbors for every node and create a dictionary { node_addr : num_neighbors }
            nodes_todo_nbrs = dict(zip(nodes_todo, [0] * len(nodes_todo)))
            for nt in nodes_todo:
                for nbr in ret.keys():
                    if distance_matrix[addresses.index(nt)][addresses.index(nbr)] > 0.0:
                        nodes_todo_nbrs[nt] += 1
            # sort by number of neighbors
            sorted_by_nbrs = sorted(nodes_todo_nbrs.items(), key=lambda kv: kv[1])
            # The last in the list is the node with the most neighbors
            node_addr, num_nbrs = sorted_by_nbrs[-1]
            # get coordinates and distances of all neighbors of node 'node_addr'
            coords = []
            distances = []
            for nbr in ret.keys():
                d = distance_matrix[addresses.index(node_addr)][addresses.index(nbr)]
                coords.append(ret[nbr])
                distances.append(d)
            if node_addr in last:
                initial_guess = last[node_addr]
            else:
                initial_guess = ret[ynode]
            if num_nbrs > 2:
                r = minimize(errfun, initial_guess, args=(coords, distances), method='CG')
            else:
                print("Using penalty term for locating", node_addr, ". nbrs:", num_nbrs)
                r = minimize(errfun_penalty, initial_guess, args=(coords, distances), method='CG')
            ret[node_addr] = np.array(r.x)

            # list of nodes that are not located yet
            nodes_todo = [addr for addr in addresses if addr not in ret.keys()]
        return ret


def pretty_print_matrix(mat, addr):
    #                 abcd |
    sys.stdout.write("       ")
    for a in addr:
        sys.stdout.write("  {}  | ".format(a))
    print()
    for row in range(len(addr)):
        sys.stdout.write("{} |{}".format(addr[row], '.........|' * (row + 1)))
        for col in range(row + 1, len(addr)):
            sys.stdout.write("{:_>+8.2f}_|".format(mat[row][col]))
        print("\n")
    print()


#######################################################################
#                  START LOG FILE ANALYSIS                            #
#######################################################################
if __name__ == "__main__":
    if len(sys.argv) is not 2:
        print("Usage: {} <filename>".format(sys.argv[0]))
        sys.exit(1)

    # 1.
    print("--- 1 ---\n")
    # Provide reference positions to compare measured distances to real distances
    # and to compare calculated positions to real positions.
    # Each position is a vector of (x, y, z) coordinates on a map. The unit is [cm]
    reference_pos_lab = dict()
    reference_pos_lab['a523'] = (143.0, 312.0, 175.0)
    reference_pos_lab['9d23'] = (425.0, 310.0, 175.0)
    reference_pos_lab['8e03'] = (412.0, 621.0, 175.0)
    reference_pos_lab['9f23'] = (312.0, 978.0, 175.0)
    reference_pos_lab['34c6'] = (132.0, 980.0, 175.0)
    reference_pos_lab['11c6'] = (160.0, 626.0, 176.0)
    reference_pos_lab['4010'] = (594.0, 670.0, 193.0)
    reference_pos_lab['4443'] = (392.0,  44.0, 196.0)
    reference_pos_lab['4210'] = (592.0, 440.0, 193.0)

    # 2.
    print("--- 2 ---\n")
    # create class instance
    # a5.23 ... address of sink node
    # 9d.23 ... address of x-axis reference node
    # 34.c6 ... address of y-axis reference node
    print("Analyzing log file: {}".format(sys.argv[1]))
    analyzer = LogfileAnalyzer(sys.argv[1], reference_pos_lab, 'a523', '9d23', '34c6')

    # 3.
    print("--- 3 ---\n")
    # Read log file. Addresses that are not listed in the reference positions will be skipped
    analyzer.readfile_to_matrix(verbose=True)
    print(analyzer)

    # 4.
    print("--- 4 ---\n")
    # calculate the difference between the real distance of a link
    # and the average of all measurements at that link.
    # It will calculate a suggested 'correction distance' too, in case the nodes have not been calibrated.
    print("\nError [m] to average distances")
    analyzer.last_position_as_initial_position = False
    e = analyzer.get_avg_distance_error()
    pretty_print_matrix(e, analyzer.network_addresses)
    # print(analyzer.network_addresses)
    # for k, v in analyzer.distance_correction.items():
    #     print("{} : {:10.4f}".format(k, v))
    # for y in range(0, len(analyzer.network_addresses)):
    #     for x in range(0, len(analyzer.network_addresses)):
    #         sys.stdout.write("{: >-10.4f}, ".format(np.abs(e[x, y])))
    #     print()

    # plot measurements of each link and compare to actual distance
    for f in range(len(analyzer.network_addresses)):
        for t in range(f + 1, len(analyzer.network_addresses)):
            node_from = analyzer.network_addresses[f]
            node_to = analyzer.network_addresses[t]
            dist1 = analyzer.get_distances_single_node(node_from, node_to)
            ref1 = analyzer.reference_distances[analyzer.network_addresses.index(node_from)][analyzer.network_addresses.index(
                node_to)]
            plt.figure(0)
            plt.title("from {} to {}".format(node_from, node_to))
            plt.xlabel("measurement number")
            plt.ylabel("distance [m]")
            plt.plot(dist1)
            plt.plot([ref1]*len(dist1), 'r')
            plt.legend(['measured', 'real'], loc='upper right')
            plt.savefig("distances_{}_{}.png".format(node_from, node_to), dpi=600, format='png')
            plt.close(0)
            # plt.show()

    # OPTIONAL!
    print("--- OPTIONAL ---\n")
    # IF the node were not properly calibrated, one can use the calculated correction value to
    # compensate for this systematic error.
    # The 'correction' parameter must be set true.
    analyzer.readfile_to_matrix(correction=True, verbose=False)
    # The average error should now be much smaller, compared to previous measurements
    print("\nAfter correcting systematic errors:")
    e = analyzer.get_avg_distance_error()
    pretty_print_matrix(e, analyzer.network_addresses)
    # Plot all distance measurements of each link again
    for f in range(len(analyzer.network_addresses)):
        for t in range(f + 1, len(analyzer.network_addresses)):
            node_from = analyzer.network_addresses[f]
            node_to = analyzer.network_addresses[t]
            dist1 = analyzer.get_distances_single_node(node_from, node_to)
            ref1 = analyzer.reference_distances[analyzer.network_addresses.index(node_from)][analyzer.network_addresses.index(
                node_to)]
            plt.figure(0)
            plt.title("from {} to {} (corrected)".format(node_from, node_to))
            plt.xlabel("measurement number")
            plt.ylabel("distance [m]")
            plt.plot(dist1)
            plt.plot([ref1]*len(dist1), 'r')
            plt.legend(['measured', 'real'], loc='upper right')
            plt.savefig("distances_{}_{}_corrected.png".format(node_from, node_to), dpi=600, format='png')
            plt.close(0)
    #         # plt.show()

    # 5.
    print("--- 5 ---\n")
    # Calculate the positions of the nodes based on their distance measurements
    # This may take a long time, depending on the number of measurements in the log file
    print("\nAverage positions")
    analyzer._calc_positions()
    ap = analyzer.get_average_position()
    print()
    for k, v in ap.items():
        print("Address {} is at ({: >+8.3f}, {: >+8.3f})".format(k, v[0], v[1]))
    # Calculate the difference between each real position and the calculated position
    p_err = analyzer.get_position_errors()

    # print suspiciously high position errors and their distance matrices
    print("--- 5.1 ---\n")
    threshold_cm = 100.0
    for address, error_list in p_err.items():
        for idx in range(len(error_list)):
            if error_list[idx] > threshold_cm:
                print("Error {} of node {} is {}cm".format(idx, address, error_list[idx]))
                pretty_print_matrix(analyzer.distances[idx], analyzer.network_addresses)

    analyzer.plot_all_positions(analyzer.network_addresses, plot_real=True)

    # 6.
    print("--- 6 ---\n")
    # Plot the CDF of each node's position error
    for a in analyzer.network_addresses:
        print("cdf {}".format(a))
        analyzer.plot_cdf(a)
        analyzer.plot_cdf_and_error(a)

    # Plot the CDF of all position estimates
    analyzer.plot_total_cdf(exclude=['a523', '9d23'], show=True)

    analyzer.save_to_file()
