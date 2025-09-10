#-------------------------------------------------------------------------------------------------------------------------------
# Arquivo  : Farthest_Insertion.py
# Conteúdo : Trabalho de Implementação - Parte I: Heurísticas Construtivas para o TSP
# Autor    : Selene Melo Andrade

# Constructive Heuristic: Farthest Insertion

#------------------ Bibliotecas -----------------------------------------------------------------------------------------------------

import math
import os
import glob
import argparse

# ---------------- Distâncias -----------------------------------------------------------------------------------------------------

def nint(x):

    return int(round(x))

def dist_euc2d(a, b):

    # a, b: tuplas (x, y)
    xd = a[0] - b[0]
    yd = a[1] - b[1]
    return nint(math.hypot(xd, yd))

def dist_att(a, b):

    # a, b: tuplas (x, y)
    xd = a[0] - b[0]
    yd = a[1] - b[1]
    r = math.sqrt((xd*xd + yd*yd) / 10.0)
    t = nint(r)
    if t < r:
        return t + 1
    else:
        return t

# ---------------- Parser TSPLIB ----------------------------------------------------------------------------------------------------

#   Lê um arquivo TSPLIB.tsp e retorna:
#       name (str) -> nome da instância
#       ewt  (str) -> EDGE_WEIGHT_TYPE (EUC_2D ou ATT)
#       pts  (list[(x,y)]) -> lista de coordenadas

def parse_tsp(path):

    with open(path, "r") as f:
        lines = [ln.strip() for ln in f]

    header = {}   
    coords = []  
    reading = False

    for ln in lines:
        if not ln:
            continue

        up = ln.upper()

        if up.startswith("NODE_COORD_SECTION"):
            reading = True
            continue

        if up.startswith("EOF"):
            break

        if reading:
            parts = ln.split()
            if len(parts) >= 3:
                idx = int(float(parts[0]))
                x = float(parts[1])
                y = float(parts[2])
                coords.append((idx, x, y))
        else:
            if ":" in ln:
                k, v = ln.split(":", 1)
                header[k.strip().upper()] = v.strip()
            else:
                parts = ln.split()
                if len(parts) >= 2:
                    header[parts[0].strip().upper()] = " ".join(parts[1:]).strip()

    coords.sort(key=lambda t: t[0])   # ordena por índice
    pts = [(x, y) for (_, x, y) in coords]
    ewt = (header.get("EDGE_WEIGHT_TYPE") or "").strip().upper()
    name = header.get("NAME", os.path.basename(path))

    return name, ewt, pts

def get_dist_func(ewt):

    if ("ATT" in (ewt or "").upper()):
        return dist_att, "ATT"
    return dist_euc2d, "EUC_2D"

# ---------------- Farthest Insertion ----------------------------------------------------------------------------------------------------

def tour_length(tour, pts, dist):

    total = 0
    n = len(tour)
    for i in range(n):
        a = tour[i]
        b = tour[(i + 1) % n]
        total += dist(pts[a], pts[b])
    return total

def farthest_insertion(pts, dist):

    n = len(pts)
    if n < 2:
        return list(range(n))

    # Escolher par mais distante
    far_i, far_j, far_d = 0, 1, -1
    for i in range(n):
        for j in range(i + 1, n):
            d = dist(pts[i], pts[j])
            if d > far_d:
                far_d = d
                far_i, far_j = i, j

    tour = [far_i, far_j]
    remaining = set(range(n)) - set(tour)
    tour.append(tour[0])  # ciclo temporário

    # Inserir nós até completar
    while remaining:
        # escolher nó mais distante do ciclo
        best_k, best_d = None, -1
        for k in remaining:
            dmin = min(dist(pts[k], pts[c]) for c in tour[:-1])
            if dmin > best_d:
                best_d, best_k = dmin, k

        # encontrar posição de inserção que minimiza incremento
        best_pos, best_inc = None, float("inf")
        for i in range(len(tour) - 1):
            a, b = tour[i], tour[i + 1]
            inc = dist(pts[a], pts[best_k]) + dist(pts[best_k], pts[b]) - dist(pts[a], pts[b])
            if inc < best_inc:
                best_inc, best_pos = inc, i + 1

        tour.insert(best_pos, best_k)
        remaining.remove(best_k)

    tour.pop()  # remove duplicado

    return tour

# ---------------- Saída TSPLIB TOUR ----------------------------------------------------------------------------------------------------

def save_tour(tour, pts, dist, name, etype, tsp_path, outdir):

    L = tour_length(tour, pts, dist)
    base = os.path.splitext(os.path.basename(tsp_path))[0]
    os.makedirs(outdir, exist_ok=True)
    out_path = os.path.join(outdir, f"{base}_farthest_insertion.tour")

    with open(out_path, "w") as f:
        f.write(f"NAME : {name}\n")
        f.write("TYPE : TOUR\n")
        f.write("COMMENT : Farthest Insertion\n")
        f.write(f"DIMENSION : {len(pts)}\n")
        f.write(f"EDGE_WEIGHT_TYPE : {etype}\n")
        f.write(f"TOUR_LENGTH : {L}\n")
        f.write("TOUR_SECTION\n")
        for v in tour:
            f.write(str(v + 1) + "\n")  # 1-based
        f.write("-1\nEOF\n")

    return out_path

# ---------------- Execução em lote ----------------------------------------------------------------------------------------------------

def run_on_instances(paths, save, outdir):

    for tsp_path in paths:
        if not os.path.exists(tsp_path):
            print(f"[AVISO] Arquivo não encontrado: {tsp_path}")
            continue

        name, ewt, pts = parse_tsp(tsp_path)
        dist, etype = get_dist_func(ewt)

        tour = farthest_insertion(pts, dist)
        L = tour_length(tour, pts, dist)

        # imprime resultado simples
        print(f"Instância: {name}")
        print(f"  Tipo de distância: {etype}")
        print(f"  Nº de cidades: {len(pts)}")
        print(f"  Comprimento do tour: {L}\n")

        if save:
            save_tour(tour, pts, dist, name, etype, tsp_path, outdir)

# ---------------- Main ----------------------------------------------------------------------------------------------------

def main():

    parser = argparse.ArgumentParser(description="Farthest Insertion para TSP (EUC_2D/ATT).")
    parser.add_argument("files", nargs="*", help="Arquivos .tsp")
    parser.add_argument("--glob", dest="glob_pat", default="", help="Padrão (ex.: './*.tsp' ou './**/*.tsp')")
    parser.add_argument("--save", action="store_true", help="Salvar .tour no formato TSPLIB")
    parser.add_argument("--outdir", default="./tours_farthest_insertion", help="Pasta de saída (--save)")
    args = parser.parse_args()

    paths = []
    if args.glob_pat:
        paths.extend(sorted(glob.glob(args.glob_pat, recursive=True)))
    if args.files:
        paths.extend(args.files)

    if not paths:
        parser.error("Nenhum arquivo informado. Use arquivos .tsp e/ou --glob.")

    run_on_instances(paths, save=args.save, outdir=args.outdir)

if __name__ == "__main__":
    main()
