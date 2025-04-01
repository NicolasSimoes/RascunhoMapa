import math
import pandas as pd
import folium
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def haversine_distance(coord1, coord2):
    R = 6371  
    lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
    lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    return R * c

def create_distance_matrix(points):
    n = len(points)
    matrix = []
    for i in range(n):
        row = []
        for j in range(n):
            distance = haversine_distance(points[i], points[j]) * 1000  # em metros
            row.append(int(distance))
        matrix.append(row)
    return matrix

def solve_tsp(distance_matrix):
    n = len(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)  
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        index = routing.Start(0)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
        return route
    else:
        return None

# Leitura do arquivo CSV
df = pd.read_csv('dbPromotoresTeste.csv', encoding="ISO-8859-1", sep=";")
df['LATITUDE CASA']  = pd.to_numeric(df['LATITUDE CASA'], errors='coerce')
df['LONGITUDE CASA'] = pd.to_numeric(df['LONGITUDE CASA'], errors='coerce')
df['LATITUDE']       = pd.to_numeric(df['LATITUDE'], errors='coerce')
df['LONGITUDE']      = pd.to_numeric(df['LONGITUDE'], errors='coerce')

if 'NOME FANTASIA' not in df.columns:
    print("A coluna 'NOME FANTASIA' não foi encontrada. Verifique o nome da coluna que contém o nome da loja.")

# Certifica que a coluna TIPO exista e remove linhas com dados faltantes relevantes
if 'TIPO' not in df.columns:
    print("A coluna 'TIPO' não foi encontrada. Verifique se o arquivo possui essa coluna.")
df = df.dropna(subset=['PROMOTOR', 'LATITUDE', 'LONGITUDE', 'TIPO'])

# Cria um dicionário com as informações dos promotores: casa e TIPO
promotores = df['PROMOTOR'].unique()
promoter_info = {}
for promotor in promotores:
    df_prom = df[df['PROMOTOR'] == promotor]
    home_coords = [df_prom.iloc[0]['LATITUDE CASA'], df_prom.iloc[0]['LONGITUDE CASA']]
    tipo = df_prom.iloc[0]['TIPO']
    promoter_info[promotor] = {'home': home_coords, 'tipo': tipo}

# Função de atribuição considerando apenas promotores com o mesmo TIPO
def assign_promoter(store_coord, store_tipo, promoter_info):
    best = None
    best_dist = float('inf')
    for promotor, info in promoter_info.items():
        # Apenas considera se o TIPO do promotor for igual ao TIPO da loja
        if info['tipo'] != store_tipo:
            continue
        d = haversine_distance(store_coord, info['home'])
        if d < best_dist:
            best_dist = d
            best = promotor
    return best

# Atribui a cada loja um promotor otimizado conforme o TIPO
df['PROMOTOR_OTIMIZADO'] = df.apply(
    lambda row: assign_promoter([row['LATITUDE'], row['LONGITUDE']], row['TIPO'], promoter_info),
    axis=1
)

# Define cores para os promotores
cores = ['blue', 'red', 'green', 'purple', 'orange', 'darkred', 'lightred', 
         'beige', 'darkblue', 'darkgreen', 'cadetblue', 'darkpurple', 'pink', 
         'lightblue', 'lightgreen', 'gray', 'black', 'lightgray']
cores_promotor = {}
for i, promotor in enumerate(promotores):
    cores_promotor[promotor] = cores[i % len(cores)]

mapa = folium.Map(location=[-3.7424091, -38.4867581], zoom_start=13)
route_distances = {}

# Agrupa as lojas pelo promotor otimizado (já considerando TIPO)
groups = df.groupby('PROMOTOR_OTIMIZADO')
for promotor_ot, group_df in groups:
    # Recupera a casa do promotor a partir do dicionário
    home = promoter_info[promotor_ot]['home']
    
    store_data = group_df[['LATITUDE', 'LONGITUDE', 'NOME FANTASIA']].values.tolist()
    points = [home] + [(lat, lon) for lat, lon, _ in store_data]

    if len(points) == 1:
        fg = folium.FeatureGroup(name=f"Promotor {promotor_ot}", show=False)
        folium.Marker(
            location=home,
            popup=f"Casa do Promotor {promotor_ot}",
            icon=folium.Icon(color=cores_promotor[promotor_ot], icon='home', prefix='fa')
        ).add_to(fg)
        fg.add_to(mapa)
        route_distances[promotor_ot] = 0
        continue

    dist_matrix = create_distance_matrix(points)
    route = solve_tsp(dist_matrix)
    if route is None:
        fg = folium.FeatureGroup(name=f"Promotor {promotor_ot} - Rota não otimizada", show=False)
        folium.Marker(
            location=home,
            popup=f"Casa do Promotor {promotor_ot} - Rota não otimizada",
            icon=folium.Icon(color=cores_promotor[promotor_ot], icon='home', prefix='fa')
        ).add_to(fg)
        fg.add_to(mapa)
        route_distances[promotor_ot] = 0
        continue

    route_coords = [points[i] for i in route]
    total_distance = 0
    for i in range(len(route) - 1):
        total_distance += dist_matrix[route[i]][route[i+1]]
    route_distance_km = total_distance / 1000.0
    route_distances[promotor_ot] = route_distance_km

    fg = folium.FeatureGroup(name=f"Promotor {promotor_ot}", show=False)
    folium.Marker(
        location=home,
        popup=f"Casa do Promotor {promotor_ot}",
        icon=folium.Icon(color=cores_promotor[promotor_ot], icon='home', prefix='fa')
    ).add_to(fg)
    for lat, lon, nome_loja in store_data:
        folium.Marker(
            location=[lat, lon],
            popup=nome_loja,
            icon=folium.Icon(color=cores_promotor[promotor_ot])
        ).add_to(fg)
    folium.PolyLine(route_coords, color=cores_promotor[promotor_ot], weight=3, dash_array='5,10').add_to(fg)
    fg.add_to(mapa)

folium.LayerControl().add_to(mapa)
mapa.save("teste.html")
print("Mapa otimizado salvo como 'mapa_promotores_otimizado_reatribuido.html'")

with open("quilometragem_total.txt", "w", encoding="utf-8") as file:
    for promotor, distance in route_distances.items():
        file.write(f"Promotor: {promotor} - Quilometragem Total: {distance:.2f} km\n")

print("Arquivo 'quilometragem_total.txt' salvo com as quilometragens totais.")
