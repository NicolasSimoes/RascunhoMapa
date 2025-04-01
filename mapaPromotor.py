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

if 'SUPERVISOR' not in df.columns:
    print("A coluna 'SUPERVISOR' não foi encontrada. Verifique se o arquivo possui essa coluna.")
df = df.dropna(subset=['PROMOTOR', 'LATITUDE', 'LONGITUDE', 'SUPERVISOR'])

# Cria um dicionário com as informações dos promotores: casa e SUPERVISOR
promotores = df['PROMOTOR'].unique()
promoter_info = {}
for promotor in promotores:
    df_prom = df[df['PROMOTOR'] == promotor]
    home_coords = [df_prom.iloc[0]['LATITUDE CASA'], df_prom.iloc[0]['LONGITUDE CASA']]
    SUPERVISOR = df_prom.iloc[0]['SUPERVISOR']
    promoter_info[promotor] = {'home': home_coords, 'SUPERVISOR': SUPERVISOR}

# Função de atribuição considerando apenas promotores com o mesmo SUPERVISOR
def assign_promoter(store_coord, store_SUPERVISOR, promoter_info):
    best = None
    best_dist = float('inf')
    for promotor, info in promoter_info.items():
        if info['SUPERVISOR'] != store_SUPERVISOR:
            continue
        d = haversine_distance(store_coord, info['home'])
        if d < best_dist:
            best_dist = d
            best = promotor
    return best

# Atribui a cada loja um promotor otimizado conforme o SUPERVISOR
df['PROMOTOR_OTIMIZADO'] = df.apply(
    lambda row: assign_promoter([row['LATITUDE'], row['LONGITUDE']], row['SUPERVISOR'], promoter_info),
    axis=1
)

# Define cores para os promotores
cores = ['blue', 'red', 'green', 'purple', 'orange', 'darkred', 'lightred', 
         'beige', 'darkblue', 'darkgreen', 'cadetblue', 'darkpurple', 'pink', 
         'lightblue', 'lightgreen', 'gray', 'black', 'lightgray']
cores_promotor = {}
for i, promotor in enumerate(promotores):
    cores_promotor[promotor] = cores[i % len(cores)]

# Cria o mapa
mapa = folium.Map(location=[-3.7424091, -38.4867581], zoom_start=13)
route_distances = {}

# Cria os grupos de camada para os supervisores (serão duas opções conforme os valores da coluna SUPERVISOR)
supervisor_feature_groups = {}
for supervisor in df['SUPERVISOR'].unique():
    supervisor_feature_groups[supervisor] = folium.FeatureGroup(name=f"Supervisor {supervisor}", show=False)
    supervisor_feature_groups[supervisor].add_to(mapa)

# Cria os grupos de camada individuais para os promotores
# Estes continuarão aparecendo no filtro com o nome de cada promotor.
groups = df.groupby('PROMOTOR_OTIMIZADO')
for promotor_ot, group_df in groups:
    supervisor = promoter_info[promotor_ot]['SUPERVISOR']
    
    # Cria o grupo do promotor
    fg_promotor = folium.FeatureGroup(name=f"Promotor {promotor_ot}", show=False)
    
    home = promoter_info[promotor_ot]['home']
    store_data = group_df[['LATITUDE', 'LONGITUDE', 'NOME FANTASIA']].values.tolist()
    points = [home] + [(lat, lon) for lat, lon, _ in store_data]

    # Se houver apenas a casa do promotor (sem lojas)
    if len(points) == 1:
        marker_prom = folium.Marker(
            location=home,
            popup=f"Casa do Promotor {promotor_ot}",
            icon=folium.Icon(color=cores_promotor[promotor_ot], icon='home', prefix='fa')
        )
        marker_prom.add_to(fg_promotor)
        # Adiciona também no grupo do supervisor
        marker_prom_sup = folium.Marker(
            location=home,
            popup=f"Casa do Promotor {promotor_ot}",
            icon=folium.Icon(color=cores_promotor[promotor_ot], icon='home', prefix='fa')
        )
        marker_prom_sup.add_to(supervisor_feature_groups[supervisor])
        
        route_distances[promotor_ot] = 0
        fg_promotor.add_to(mapa)
        continue

    dist_matrix = create_distance_matrix(points)
    route = solve_tsp(dist_matrix)
    if route is None:
        marker_prom = folium.Marker(
            location=home,
            popup=f"Casa do Promotor {promotor_ot} - Rota não otimizada",
            icon=folium.Icon(color=cores_promotor[promotor_ot], icon='home', prefix='fa')
        )
        marker_prom.add_to(fg_promotor)
        marker_prom_sup = folium.Marker(
            location=home,
            popup=f"Casa do Promotor {promotor_ot} - Rota não otimizada",
            icon=folium.Icon(color=cores_promotor[promotor_ot], icon='home', prefix='fa')
        )
        marker_prom_sup.add_to(supervisor_feature_groups[supervisor])
        route_distances[promotor_ot] = 0
        fg_promotor.add_to(mapa)
        continue

    route_coords = [points[i] for i in route]
    total_distance = sum(dist_matrix[route[i]][route[i+1]] for i in range(len(route)-1))
    route_distance_km = total_distance / 1000.0
    route_distances[promotor_ot] = route_distance_km

    # Marcador da casa do promotor (adicionado em ambos os grupos)
    marker_home_prom = folium.Marker(
        location=home,
        popup=f"Casa do Promotor {promotor_ot}",
        icon=folium.Icon(color=cores_promotor[promotor_ot], icon='home', prefix='fa')
    )
    marker_home_prom.add_to(fg_promotor)
    marker_home_sup = folium.Marker(
        location=home,
        popup=f"Casa do Promotor {promotor_ot}",
        icon=folium.Icon(color=cores_promotor[promotor_ot], icon='home', prefix='fa')
    )
    marker_home_sup.add_to(supervisor_feature_groups[supervisor])
    
    # Marcadores das lojas (em ambos os grupos)
    for lat, lon, nome_loja in store_data:
        marker_store_prom = folium.Marker(
            location=[lat, lon],
            popup=nome_loja,
            icon=folium.Icon(color=cores_promotor[promotor_ot])
        )
        marker_store_prom.add_to(fg_promotor)
        
        marker_store_sup = folium.Marker(
            location=[lat, lon],
            popup=nome_loja,
            icon=folium.Icon(color=cores_promotor[promotor_ot])
        )
        marker_store_sup.add_to(supervisor_feature_groups[supervisor])
    
    # Linha da rota (adicionada em ambos os grupos)
    polyline_prom = folium.PolyLine(route_coords, color=cores_promotor[promotor_ot], weight=3, dash_array='5,10')
    polyline_prom.add_to(fg_promotor)
    polyline_sup = folium.PolyLine(route_coords, color=cores_promotor[promotor_ot], weight=3, dash_array='5,10')
    polyline_sup.add_to(supervisor_feature_groups[supervisor])
    
    fg_promotor.add_to(mapa)

folium.LayerControl().add_to(mapa)
mapa.save("teste.html")
print("Mapa otimizado salvo como 'teste.html'")

with open("quilometragem_total.txt", "w", encoding="utf-8") as file:
    for promotor, distance in route_distances.items():
        file.write(f"Promotor: {promotor} - Quilometragem Total: {distance:.2f} km\n")

print("Arquivo 'quilometragem_total.txt' salvo com as quilometragens totais.")
