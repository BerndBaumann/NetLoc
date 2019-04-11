#!/usr/bin/env python3
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import sys
import threading
import queue
from scipy.optimize import minimize



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


def _localize(ref, distance_matrix, addresses, last={}, sink='a523', xnode='9d23', ynode='34c6'):
    ret = dict()
    # pretty_print_matrix(distance_matrix, addresses)
    try:
        smat_idx = addresses.index(sink)
    except ValueError:
        print("Cannot calculate coordinates. Initial nodes are missing: ", sink)
        return ret
    try:
        xmat_idx = addresses.index(xnode)
    except ValueError:
        print("Cannot calculate coordinates. Initial nodes are missing: ", xnode)
        return ret
    try:
        ymat_idx = addresses.index(ynode)
    except ValueError:
        print("Cannot calculate coordinates. Initial nodes are missing: ", ynode)
        return ret
    # start with the triangle sink-xnode-ynode
    ret[sink] = np.array([ref[sink][0], ref[sink][1]])
    ret[xnode] = np.array([ret[sink][0] + distance_matrix[smat_idx][xmat_idx], ret[sink][1]])
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

def transform_result(coordinates, origin_key, x_node=None, y_node=None, ref=None):
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


def transform_to_map_view(coordinates, displacement, angle):
    transformed = {}
    for k, v in coordinates.items():
        rot = np.matrix([[np.cos(angle), np.sin(angle)],
                         [-np.sin(angle), np.cos(angle)]])
        tmp = rot * np.matrix(v).T + displacement.T
        transformed[k] = (float(tmp[0]), float(tmp[1]))
    return transformed


def calc_reference_distances(reference_positions, network_addresses):
    reference_distances = np.zeros((len(network_addresses), len(network_addresses)))
    for row in range(0, len(network_addresses)):
        node_from = reference_positions[network_addresses[row]]
        for col in range(row + 1, len(network_addresses)):
            node_to = reference_positions[network_addresses[col]]
            reference_distances[row][col] = np.linalg.norm(np.array(node_from) - np.array(node_to))
            reference_distances[col][row] = reference_distances[row][col]
    return reference_distances / 100.0


def project_distances(distance_matrix, addresses, reference_coordinates):
    # projected distance = sqrt( (measured distance)^2 - z_diff^2 )
    projected = np.zeros((len(distance_matrix), len(distance_matrix)))
    for row in range(0, len(distance_matrix)):
        for col in range(row + 1, len(distance_matrix)):
            if distance_matrix[row, col] == 0.0:
                continue
            z_diff = reference_coordinates[addresses[row]][2] - reference_coordinates[addresses[col]][2]
            z_diff /= 100.0  # cm to m
            projected[row, col] = np.sqrt(np.square(distance_matrix[row, col]) - np.square(z_diff))
            projected[col, row] = projected[row, col]
    projected_r = dict()
    for k, v in reference_coordinates.items():
        projected_r[k] = (v[0], v[1])
    return projected, projected_r


def cleanUpMatrix(current_matrix, current_addresses):
    del_a = []
    for addr in current_addresses:
        idx = current_addresses.index(addr)
        tmp = 0.0
        for dist in current_matrix[idx]:
            tmp += dist
        if tmp == 0.0:
            del_a.append(addr)
            print("Address to be deleted:", addr)
    for addr in del_a:
        idx = current_addresses.index(addr)
        # delete row and column in matrix

    return current_matrix, current_addresses


def watch_log_file(pos_q, fp, references, skip_addresses=[]):
    current_seqnum = 1
    current_matrix = []
    current_addresses = []
    last_coordinates = {}
    while True:
        line = fp.readline()
        if "RPLUWB" in line:
            # 1.
            # Parse line and extract neighbor and distance information
            startidx = line.index("RPLUWB")
            line = line[startidx:]
            measurement = line.strip().split(',')
            sender = measurement[1].strip()
            if sender in skip_addresses:
                continue
            seqnum = int(measurement[2])
            new_edges = []  # list of tuples [(nodeA, nodeB, distance), ...]
            for new_edge in list(zip(measurement[3::2], measurement[4::2])):
                neighbor = new_edge[0].strip()
                distance = float(new_edge[1])
                if neighbor in skip_addresses:
                    continue
                if neighbor != '0000' and distance > 0.0:
                    new_edges.append((sender, neighbor, distance))
            if not new_edges:
                continue

            # 2.
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
                    current_matrix[row_index][col_index] = \
                        (0.9 * new_edge[2] + 0.1 * current_matrix[row_index][col_index])
                else:
                    current_matrix[row_index][col_index] = new_edge[2]
                current_matrix[col_index][row_index] = current_matrix[row_index][col_index]

            # 2.1 clean up disconnected nodes
            # current_matrix, current_addresses = cleanUpMatrix(current_matrix, current_addresses)

            # 3.
            # calculate the positions of the nodes based on the collected
            # distance measurements
            try:
                np_mat = np.array(current_matrix, dtype=[('len', np.float)])
                graph = nx.from_numpy_array(np_mat)
                graph = nx.relabel_nodes(graph, dict(zip(range(len(current_addresses)), current_addresses)))
                coordinates = _localize(references, current_matrix, current_addresses, last_coordinates)
                last_coordinates = coordinates
                pos_q.put((seqnum, coordinates, graph.adj))
                print("\n--- {} ---\n".format(seqnum, coordinates))
            except KeyError as ke:
                print("KeyError: {}. Current sequence number: {}".format(ke, current_seqnum))
            except ValueError as ve:
                print("ValueError: ", ve)

        else:
            time.sleep(0.01)


def matrixupdate(iteration, pos_q, ax, refs):
    # print("iteration {}".format(iteration))
    if not pos_q.empty():
        num, pos, adj = pos_q.get()
        xpos_ = []
        ypos_ = []
        label_ = []
        color = []
        for k, v in pos.items():
            xpos_.append(v[0])
            ypos_.append(v[1])
            label_.append(k)
            if k == 'a523':
                color.append('r')
            else:
                color.append('b')

        ax.clear()
        ax.set_title("Measurement #{}".format(num))
        ax.grid(True)
        for label, x, y, in zip(label_, xpos_, ypos_):
            ax.annotate(label, xy=(x, y), xytext=(-20, 20),
                        textcoords='offset points', ha='center', va='center',
                        bbox=dict(boxstyle='round, pad=0.5', fc='yellow', alpha=0.5),
                        arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
        ax.scatter(xpos_, ypos_, s=100, c=color, alpha=0.5)

        for a in range(0, len(label_)):
            for b in range(a + 1, len(label_)):
                try:
                    if adj[label_[a]][label_[b]]['len'] > 0.0:
                        # plot edge between node label_[a] and node label_[b]
                        k, d = np.polyfit([xpos_[a], xpos_[b]], [ypos_[a], ypos_[b]], 1)
                        xline = np.linspace(xpos_[a], xpos_[b], 3)
                        yline = k*xline + d
                        ax.plot(xline, yline, 'b-')
                        ax.annotate("{:.2f}".format(adj[label_[a]][label_[b]]['len']), xy=(xline[1], yline[1]))
                except KeyError:
                    pass

        # draw room
        # wall0 = 580.0  # length in cm
        # wall1 = 770.0  # length in cm
        wall0 = 630.0
        wall1 = 1000.0
        room_corners = np.array([[0, 0], [wall0, 0], [wall0, wall1], [0, wall1], [0, 0]])
        ax.plot(room_corners[:, 0], room_corners[:, 1], 'k-')
        # draw reference positions
        ax.scatter(refs[:, 0], refs[:, 1], s=200, c='g', marker='x', alpha=0.3)


# reference_pos_office = dict()
# reference_pos_office['a523'] = (314.0, 190.0)  # 155.0)
# reference_pos_office['9d23'] = (493.0, 190.0)  # 155.0)
# reference_pos_office['8e03'] = (400.0, 370.0)  # 155.0)
# reference_pos_office['9f23'] = (100.0, 370.0)  # 155.0)
# reference_pos_office['34c6'] = (100.0, 670.0)  # 155.0)
# reference_pos_office['11c6'] = (400.0, 670.0)  # 155.0)
# reference_pos_office['4010'] = (160.0, 215.0)  # 155.0)
# reference_pos_office['4210'] = (155.0, 530.0)  # 155.0)
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
# skip = ['4010', '4210', '4443']
skip = []
fp_ = open(sys.argv[1], 'r')
position_q = queue.Queue()
reader = threading.Thread(target=watch_log_file, args=(position_q, fp_, reference_pos_lab, skip), name="Reader")
reader.start()

fig0 = plt.figure(0)
ax0 = fig0.add_subplot(111)
ax0.grid(True)
refs0 = np.array([[v[0], v[1]] for k, v in reference_pos_lab.items()])
animation = FuncAnimation(fig0, matrixupdate, fargs=(position_q, ax0, refs0), interval=100)
plt.show()
