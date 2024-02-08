import heapq
import timeit
import tracemalloc


class Graph(object):
    def __init__(self, graph_dict=None):
        if graph_dict is None:
            graph_dict = {}
        self._graph_dict = graph_dict
        self.invalid_person = False  # Nieuwe variabele voor invalide status

    def set_invalid_status(self, status):
        self.invalid_person = status

    def can_use_lift(self, edge):
        if edge[0].startswith("L") and not self.invalid_person:
            return False
        return True

    def readGraph(self, filePath):
        inputFile = open(filePath, 'r')
        data = str()
        iRegel = 0
        for regel in inputFile:
            if iRegel > 0:
                data += ", "
            data += regel.strip()
            iRegel += 1
        inputFile.close()

        self._graph_dict = eval("{" + data + "}")

    def edges(self, vertice):
        """ returns a list of tuples (neighbor, distance) """
        return self._graph_dict[vertice]

    def all_vertices(self):
        """ returns the vertices of a graph as a set """
        return set(self._graph_dict.keys())

    def all_edges(self):
        """ returns the edges of a graph as a list of tuples (vertex1, vertex2, distance) """
        return self.__generate_edges()

    def add_vertex(self, vertex):
        """ If the vertex "vertex" is not
            in self._graph_dict, a key "vertex" with an empty
            list as a value is added to the dictionary.
            Otherwise nothing has to be done.
        """
        if vertex not in self._graph_dict:
            self._graph_dict[vertex] = []

    def add_edge(self, edge, distance):
        """ assumes that edge is of type set, tuple or list;
            between two vertices can be multiple edges!
        """
        edge = set(edge)
        vertex1, vertex2 = tuple(edge)
        self._graph_dict[vertex1].append((vertex2, distance))
        self._graph_dict[vertex2].append((vertex1, distance))

    def __generate_edges(self):
        """ A method generating the edges of the
            graph "graph". Edges are represented as tuples
            (vertex1, vertex2, distance)
        """
        edges = []
        for vertex in self._graph_dict:
            for neighbor, distance in self._graph_dict[vertex]:
                if (neighbor, vertex, distance) not in edges and (vertex, neighbor, distance) not in edges:
                    edges.append((vertex, neighbor, distance))
        return edges

    def __iter__(self):
        self._iter_obj = iter(self._graph_dict)
        return self._iter_obj

    def __next__(self):
        """ allows us to iterate over the vertices """
        return next(self._iter_obj)

    def __str__(self):
        res = "vertices: "
        for k in self._graph_dict:
            res += str(k) + " "
        res += "\nedges: "
        for edge in self.__generate_edges():
            res += str(edge) + " "
        return res


def dijkstra(graph, start, end):
    queue = [(0, start, [])]
    visited = set()

    while queue:
        (cost, node, path) = heapq.heappop(queue)

        if node not in visited:
            visited.add(node)
            path = path + [node]

            if node == end:
                return path, cost

            for neighbor, distance in graph.edges(node):
                if neighbor in graph.all_vertices() and neighbor not in visited:
                    if graph.can_use_lift((node, neighbor, distance)):
                        heapq.heappush(queue, (cost + distance, neighbor, path))

    return None, None

def find_shortest_path(graph, locations):
    path = []
    total_distance = 0

    # Filter out group nodes
    locations = [loc for loc in locations if '.' in loc]

    for i in range(len(locations) - 1):
        start_node = locations[i]
        end_node = locations[i + 1]
        shortest_path, distance = dijkstra(graph, start_node, end_node)

        if shortest_path:
            path += shortest_path[:-1]  # Exclude the duplicate node
            total_distance += distance

    path.append(locations[-1])
    return path, total_distance

def find_nearest_exit(graph, current_location):
    graph.set_invalid_status(False) #Invalide mensen ook via trap [Take the L]
    exit_nodes = [node for node in graph.all_vertices() if node.startswith("N")]

    if not exit_nodes:
        print("Er zijn geen nooduitgangen gedefinieerd.")
        return

    distances = []
    for exit_node in exit_nodes:
        path, distance = dijkstra(graph, current_location, exit_node)
        if path and distance is not None:
            distances.append((exit_node, distance))

    if not distances:
        print("Kan geen route naar een nooduitgang vinden vanaf de opgegeven locatie.")
        return

    nearest_exit, nearest_distance = min(distances, key=lambda x: x[1])

    print(f"\nDe dichtstbijzijnde nooduitgang is: {nearest_exit}")
    print(f"De afstand naar de nooduitgang is: {nearest_distance}m")
    print(f"De route naar de nooduitgang is: {' - '.join(dijkstra(graph, current_location, nearest_exit)[0])}")

def read_schedule(file_path):
    schedule = []
    with open(file_path, 'r') as schedule_file:
        for line in schedule_file:
            line = line.strip()
            if line:
                time, location = line.split(' - ')
                schedule.append((time, location))

    return schedule

def calculate_route_for_schedule(graph, schedule):
    if not schedule:
        print("Het rooster is leeg.")
        return

    start_location = input("\nGeef de startlocatie (bijv. Hoofdingang of Fietsenstalling): ")
    if start_location == "Hoofdingang":
        start_location = "N0.4"
        previous_location = start_location
    elif start_location == "Fietsenstalling":
        start_location = "001"
        previous_location = start_location  # Voeg een variabele toe om de vorige locatie bij te houden
    else:
        print("Je moet wel via een ingang naar binnen dommie!")
        return
    
    ##
    tracemalloc.start()
    start_time = timeit.default_timer()
    ##


    for i in range(len(schedule)):
        current_time, current_location = schedule[i]

        # Controleer of de huidige locatie anders is dan de vorige locatie
        if current_location != previous_location:
            print(f"\n{current_time} - {current_location}")
            shortest_path, shortest_distance = dijkstra(graph, start_location, current_location)

            if shortest_path:
                print(f"{start_location} -> {current_location}: {' - '.join(shortest_path)}")
                print(f"Route afstand: {shortest_distance}m")
            else:
                print("Er is geen route gevonden.")

        start_location = current_location  # Update de startlocatie voor de volgende iteratie
        previous_location = current_location  # Update de vorige locatie voor de volgende iteratie

    
    ##
    end_time = timeit.default_timer()
    execution_time = end_time - start_time
    print(f"\nTotale uitvoeringstijd: {execution_time} seconden")

    current, peak = tracemalloc.get_traced_memory()
    print(f"Maximaal geheugengebruik: {peak / 10**6} MB")
    tracemalloc.stop()
    ##

if __name__ == '__main__':
    g = {
        #Knooppunten Verdieping 3
        "300": [("304", 7), ("200", 5), ("B.3.225", 7), ("B.3.223", 6)],
        "304": [("300", 7), ("303", 21), ("305", 21), ("B.3.215", 18), ("B.3.208", 15), ("B.3.210", 14), ("B.3.217", 15), ("B.3.U2", 7), ("B.3.305", 11), ("B.3.300", 11), ("B.3.302", 14)],
        "305": [("N3.1", 14), ("304", 21), ("B.3.305", 10), ("B.3.300", 10), ("B.3.302", 7), ("B.3.304", 1), ("B.3.306", 1), ("B.3.308", 7), ("B.3.310", 9), ("B.3.311", 12), ("B.3.309", 13)],
        "303": [("304", 21), ("302", 21), ("B.3.215", 3), ("B.3.208", 6), ("B.3.210", 7), ("B.3.217", 6), ("B.3.209", 11), ("B.3.211", 9), ("B.3.213", 7), ("B.3.206", 10)],
        "302": [("L3.1", 3),("N3.2", 35), ("303", 21), ("301", 21), ("B.3.209", 10), ("B.3.211", 12), ("B.3.213", 14), ("B.3.206", 11), ("B.3.105", 7), ("B.3.103", 9), ("B.3.107", 8), ("B.3.U1", 7)],
        "301": [("302", 21), ("201", 5)],

        #Knooppunten Verdieping 2
        "200": [("204", 7), ("300", 5), ("100", 5), ("B.2.225", 7), ("B.2.223", 6)],
        "204": [("200", 7), ("203", 21), ("205", 21), ("B.2.215", 18), ("B.2.208", 15), ("B.2.210", 14), ("B.2.217", 15), ("B.2.U2", 7), ("B.2.305", 11), ("B.2.300", 11), ("B.2.302", 14)],
        "205": [("N2.1", 14), ("204", 21), ("B.2.305", 10), ("B.2.300", 10), ("B.2.302", 7), ("B.2.304", 1), ("B.2.306", 1), ("B.2.308", 7), ("B.2.310", 9), ("B.2.311", 12), ("B.2.309", 13)],
        "203": [("204", 21), ("202", 21), ("B.2.215", 3), ("B.2.208", 6), ("B.2.210", 7), ("B.2.217", 6), ("B.2.209", 11), ("B.2.211", 9), ("B.2.213", 7), ("B.2.206", 10)],
        "202": [("L2.1", 3),("N2.2", 35), ("203", 21), ("201", 21), ("B.2.209", 10), ("B.2.211", 12), ("B.2.213", 14), ("B.2.206", 11), ("B.2.105", 7), ("B.2.103", 9), ("B.2.107", 8), ("B.2.U1", 7)],
        "201": [("202", 21), ("301", 5), ("101", 5)],

        #Knooppunten Verdieping 1
        "100": [("104", 7), ("200", 5), ("B.1.225", 7), ("B.1.223", 6), ("000", 5)],
        "104": [("100", 7), ("103", 21), ("105", 21), ("B.1.215", 18), ("B.1.208", 15), ("B.1.210", 14), ("B.1.217", 15), ("B.1.U2", 7), ("B.1.305", 11), ("B.1.300", 11), ("B.1.302", 14)],
        "105": [("N1.1", 14), ("104", 21), ("B.1.305", 10), ("B.1.300", 10), ("B.1.302", 7), ("B.1.304", 1), ("B.1.306", 1), ("B.1.308", 7), ("B.1.310", 9), ("B.1.311", 12), ("B.1.309", 13)],
        "103": [("104", 21), ("102", 21), ("B.1.215", 3), ("B.1.208", 6), ("B.1.210", 7), ("B.1.217", 6), ("B.1.209", 11), ("B.1.211", 9), ("B.1.213", 7), ("B.1.206", 10)],
        "102": [("L1.1", 3),("N2.2", 35), ("103", 21), ("101", 21), ("B.1.209", 10), ("B.1.211", 12), ("B.1.213", 14), ("B.1.206", 11), ("B.1.105", 7), ("B.1.103", 9), ("B.1.107", 8), ("B.1.U1", 7)],
        "101": [("102", 21), ("201", 5), ("001", 5)],

        #Knooppunten Begane grond
        "000": [("004", 7), ("100", 5), ("B.0.233", 8), ("B.0.231", 8), ("B.0.229", 7), ("B.0.227", 7), ("B.0.225", 6)],
        "004": [("000", 7), ("003", 18), ("N0.1", 35), ("B.0.202", 9), ("B.0.219", 12), ("B.0.223", 5), ("B.0.204", 2), ("B.0.305", 10), ("B.0.307", 13), ("B.0.309", 16), ("B.0.U2", 4)],
        "003": [("004", 18), ("002", 18), ("B.0.211", 3), ("B.0.209", 14), ("B.0.223", 18), ("B.0.219", 8), ("B.0.202", 9), ("B.0.200", 0), ("B.0.215", 0)],
        "002": [("L0.1", 3), ("003", 18), ("001", 12), ("006", 35), ("H.0.001", 35), ("B.0.U1", 7), ("B.0.201", 3), ("B.0.104", 14), ("B.0.107", 14), ("B.0.205", 0), ("B.0.209", 8), ("B.0.211", 17)],
        "001": [("002", 12), ("006", 28), ("B.0.201", 14), ("B.0.U1", 9)],
        "006": [("001", 28), ("002", 35), ("K.0.001", 7), ("H.0.001", 14)],

        ##LIFTEN
        "L0.1": [("002", 3), ("L1.1", 0)],
        "L1.1": [("102", 3), ("L2.1", 0), ("L0.1", 0)],
        "L2.1": [("202", 3), ("L1.1", 0), ("L3.1", 0)],
        "L3.1": [("302", 3), ("L2.1", 0)],


        ###NOODUITGANGEN

        #Verdieping 3
        "N3.1": [("305", 14), ("B.3.304", 13), ("B.3.306", 13), ("B.3.308", 7), ("B.3.310", 5), ("B.3.311", 2), ("B.3.309", 2)],
        "N3.2": [("302", 35)],

        #Verdieping 2
        "N2.1": [("205", 14), ("B.2.304", 13), ("B.2.306", 13), ("B.2.308", 7), ("B.2.310", 5), ("B.2.311", 2), ("B.2.309", 2)],
        "N2.2": [("202", 35)],

        #Verdieping 1
        "N1.1": [("105", 14), ("B.1.304", 13), ("B.1.306", 13), ("B.1.308", 7), ("B.1.310", 5), ("B.1.311", 2), ("B.1.309", 2)],
        "N1.2": [("102", 35)],

        #Verdieping Begane grond
        "N0.1": [("004", 35), ("B.0.304", 23), ("B.0.305", 18), ("B.0.307", 16), ("B.0.309", 6)], 
        "N0.2": [("002", 35)], 
        "N0.3": [("006", 28), ("K.0.001", 7)],
        "N0.4": [("002", 35), ("006", 21), ("001", 26), ("H.0.001", 7)],

        ##Begane grond

        #Lokalen Begane grond bij 000
        "B.0.233": [("000", 8)],
        "B.0.231": [("000", 8)],
        "B.0.229": [("000", 7)],
        "B.0.227": [("000", 7)],
        "B.0.225": [("000", 6)],
        
        #Lokalen bij 004 en N.01
        "B.0.204": [("004", 2), ("N0.1", 23)],
        "B.0.305": [("004", 10), ("N0.1", 18)],
        "B.0.307": [("004", 13), ("N0.1", 16)],
        "B.0.309": [("004", 16), ("N0.1", 6)],
        "B.0.U2": [("004", 4)],

        #Lokalen bij 003 en 004
        "B.0.223": [("003", 18), ("004", 5)],
        "B.0.219": [("003", 8), ("004", 12)],
        "B.0.202": [("003", 9), ("004", 9)],
        "B.0.200": [("003", 0)],
        "B.0.215": [("003", 0)],

        #Lokalen bij 003 en 002
        "B.0.211": [("003", 3), ("002", 17)],
        "B.0.209": [("003", 14), ("002", 8)],
        "B.0.205": [("002", 0)],
        "B.0.107": [("002", 14)],
        "B.0.104": [("002", 14)],

        #Lokaal tussen 002 en 001
        "B.0.201": [("002", 3), ("001", 14)],
        "B.0.U1": [("002", 7), ("001", 9)],

        #KANTINE en Receptie bij 006 Nooduitgang en 002
        "K.0.001": [("006", 7), ("N.03", 7)], #KANTINE
        "H.0.001": [("006",14), ("N.04", 7), ("002", 35)], #RECEPTIE





        ##VERDIEPING 3

        #Lokalen Verdieping 3 Tussen 302 en 303:
        "B.3.209": [("302", 10), ("303", 11)],
        "B.3.211": [("302", 12), ("303", 9)],
        "B.3.213": [("302", 14), ("303", 7)],
        "B.3.206": [("302", 11), ("303", 10)],

        #Lokalen Verdieping 3 Bij 302
        "B.3.105": [("302", 7)],
        "B.3.103": [("302", 9)],
        "B.3.107": [("302", 8)],
        "B.3.U1": [("302", 7)],

        #Lokalen Verdieping 3 Tussen 303 en 304:
        "B.3.215": [("303", 3),("304", 18)],
        "B.3.208": [("303", 6),("304", 15)],
        "B.3.210": [("303", 7),("304", 14)],
        "B.3.217": [("303", 6),("304", 15)],

        #Lokalen Verdieping 3 bij 300:
        "B.3.225": [("300", 7)],
        "B.3.223": [("300", 6)],

        #Lokalen Verdieping 3 bij 304:
        "B.3.U2": [("304", 7)],

        #Lokalen Verdieping 3 Tussen 304 en 305:
        "B.3.305": [("304", 11),("305", 10)],
        "B.3.300": [("304", 11),("305", 10)],
        "B.3.302": [("304", 14),("305", 7)],

        #Lokalen Verdieping 3 tussen 305 en Nooduitgang 1:
        "B.3.304": [("305", 1), ("N3.1", 13)], 
        "B.3.306": [("305", 1), ("N3.1", 13)],
        "B.3.308": [("305", 7), ("N3.1", 7)],
        "B.3.310": [("305", 9), ("N3.1", 5)],
        "B.3.311": [("305", 12), ("N3.1", 2)],
        "B.3.309": [("305", 13), ("N3.1", 2)],

        ##VERDIEPING 2

        #Lokalen Verdieping 2 Tussen 202 en 203:
        "B.2.209": [("202", 10), ("203", 11)],
        "B.2.211": [("202", 12), ("203", 9)],
        "B.2.213": [("202", 14), ("203", 7)],
        "B.2.206": [("202", 11), ("203", 10)],

        #Lokalen Verdieping 2 Bij 202
        "B.2.105": [("202", 7)],
        "B.2.103": [("202", 9)],
        "B.2.107": [("202", 8)],
        "B.2.U1": [("202", 7)],

        #Lokalen Verdieping 2 Tussen 203 en 204:
        "B.2.215": [("203", 3),("204", 18)],
        "B.2.208": [("203", 6),("204", 15)],
        "B.2.210": [("203", 7),("204", 14)],
        "B.2.217": [("203", 6),("204", 15)],

        #Lokalen Verdieping 2 bij 200:
        "B.2.225": [("200", 7)],
        "B.2.223": [("200", 6)],

        #Lokalen Verdieping 2 bij 204:
        "B.2.U2": [("204", 7)],

        #Lokalen Verdieping 2 Tussen 204 en 205:
        "B.2.305": [("204", 11),("205", 10)],
        "B.2.300": [("204", 11),("205", 10)],
        "B.2.302": [("204", 14),("205", 7)],

        #Lokalen Verdieping 2 bij 205:
        "B.2.304": [("205", 1), ("N2.1", 13)],
        "B.2.306": [("205", 1), ("N2.1", 13)],
        "B.2.308": [("205", 7), ("N2.1", 7)],
        "B.2.310": [("205", 9), ("N2.1", 5)],
        "B.2.211": [("205", 12), ("N2.1", 2)],
        "B.2.309": [("205", 13), ("N2.1", 2)],
        
        ##VERDIEPING 1

        #Lokalen Verdieping 1 Tussen 102 en 103:
        "B.1.209": [("102", 10), ("103", 11)],
        "B.1.211": [("102", 12), ("103", 9)],
        "B.1.213": [("102", 14), ("103", 7)],
        "B.1.206": [("102", 11), ("103", 10)],

        #Lokalen Verdieping 1 Bij 102
        "B.1.105": [("102", 7)],
        "B.1.103": [("102", 9)],
        "B.1.107": [("102", 8)],
        "B.1.U1": [("102", 7)],

        #Lokalen Verdieping 1 Tussen 103 en 104:
        "B.1.215": [("103", 3),("104", 18)],
        "B.1.208": [("103", 6),("104", 15)],
        "B.1.210": [("103", 7),("104", 14)],
        "B.1.217": [("103", 6),("104", 15)],

        #Lokalen Verdieping 1 bij 100:
        "B.1.225": [("100", 7)],
        "B.1.223": [("100", 6)],

        #Lokalen Verdieping 1 bij 104:
        "B.1.U2": [("104", 7)],

        #Lokalen Verdieping 1 Tussen 104 en 105:
        "B.1.305": [("104", 11),("105", 10)],
        "B.1.300": [("104", 11),("105", 10)],
        "B.1.302": [("104", 14),("105", 7)],

        #Lokalen Verdieping 1 bij 105:
        "B.1.304": [("105", 1), ("N1.1", 13)],
        "B.1.306": [("105", 1), ("N1.1", 13)],
        "B.1.308": [("105", 7), ("N1.1", 7)],
        "B.1.310": [("105", 9), ("N1.1", 5)],
        "B.1.311": [("105", 12), ("N1.1", 2)],
        "B.1.309": [("105", 13), ("N1.1", 2)],
    }


    graph = Graph(g)

    invalid_input = input("Bent u invalide? (ja/nee): ").lower()
    if invalid_input == "ja":
        graph.set_invalid_status(True)

while True:
    print("\nKies een optie:")
    print("1. Vluchtroute berekenen")
    print("2. Kortste route tussen 2 lokalen berekenen")
    print("3. Planning volgen")
    print("4. Afsluiten")

    keuze = input("Voer het nummer van uw keuze in: ")

    if keuze == '1':
        current_location = input("\nIn welk lokaal begeeft u zich op dit moment? ")

        ##
        tracemalloc.start()
        start_time = timeit.default_timer()
        ##

        if current_location in graph.all_vertices():
            find_nearest_exit(graph, current_location)
        else:
            print("Ongeldige locatie. Voer alstublieft een geldig lokaal in.")

        
        ##
        end_time = timeit.default_timer()
        execution_time = end_time - start_time
        print(f"\nTotale uitvoeringstijd: {execution_time} seconden")

        current, peak = tracemalloc.get_traced_memory()
        print(f"Maximaal geheugengebruik: {peak / 10**6} MB")
        tracemalloc.stop()
        ##
    elif keuze == '2':
        start_location = input("\nGeef de startlocatie: ")
        end_location = input("Geef de eindlocatie: ")

        ##
        tracemalloc.start()
        start_time = timeit.default_timer()
        ##
        
        if start_location in graph.all_vertices() and end_location in graph.all_vertices():

            shortest_path, shortest_distance = dijkstra(graph, start_location, end_location)

            if shortest_path:
                print(f"De kortste route is: {' - '.join(shortest_path)}")
                print(f"De kortste afstand is: {shortest_distance}m")
            else:
                print("Er is geen route gevonden.")
        else:
            print("Ongeldige locaties opgegeven.")

        ##
        end_time = timeit.default_timer()
        execution_time = end_time - start_time
        print(f"\nTotale uitvoeringstijd: {execution_time} seconden")

        current, peak = tracemalloc.get_traced_memory()
        print(f"Maximaal geheugengebruik: {peak / 10**6} MB")
        tracemalloc.stop()
        ##

    elif keuze == '3':

            schedule_file_path = "Programma.txt"
            schedule = read_schedule(schedule_file_path)
            calculate_route_for_schedule(graph, schedule)

    elif keuze == '4':
        print("\nProgramma wordt afgesloten. Tot ziens!")
        break
    else:
        print("\nOngeldige keuze. Voer alstublieft een geldig nummer in.")
