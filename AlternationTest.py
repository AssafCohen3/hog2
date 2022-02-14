import sqlite3
import subprocess

DB_PATH = 'results.sqlite'
WEIGHTS = [1, 1.1, 2, 2.5, 5, 10, 20]
ALTERNATION_EQUAL = [1, 2, 5, 10]
ALTERNATION_DIFFERENT = [(1, 2), (1, 5), (1, 10)]
PROBLEMS_NUMBER = 100
RETRY_LIMIT = 1
ASTAR_ID = 1
MM_ID = 2
RETRY_KILLED = False


def get_connection():
    return sqlite3.connect(DB_PATH)


def create_tables(conn):
    conn.execute("""
        CREATE TABLE IF NOT EXISTS Result(
            id integer primary key autoincrement not null,
            problemNumber integer not null,
            algorithm integer not null,
            weight real not null,
            alternationRateForward integer not null,
            alternationRateBackward integer not null,
            result integer not null,
            expandedNodes integer not null,
            timeElapsed real not null
        )
    """)
    conn.commit()


def save_result(conn, problem, alg, weight, result, expanded_nodes, time_elapsed, alternation_rate_forward=0, alternation_rate_backward=0):
    print(f'saving result ({problem=}, {alg=}, {weight=}, {alternation_rate_forward=}, {alternation_rate_backward=}, {result=}, {expanded_nodes=}, {time_elapsed=})...')
    conn.execute("""
        insert into Result (problemNumber, algorithm, weight, alternationRateForward, alternationRateBackward, result, expandedNodes, timeElapsed) values (?, ?, ?, ?, ?, ?, ?, ?)""",
                 [problem, alg, weight, alternation_rate_forward, alternation_rate_backward, result, expanded_nodes, time_elapsed])
    conn.commit()


def solve_problem(problem, alg, weight, alternation_rate_forward=0, alternation_rate_backward=0):
    for i in range(0, RETRY_LIMIT):
        process = subprocess.run(['./bin/release/bidirectional', '-mytest', str(problem), str(alg), str(round(weight, 2)), str(alternation_rate_forward),
                                  str(alternation_rate_backward)],
                                 stdout=subprocess.PIPE,
                                 stdin=subprocess.PIPE,
                                 encoding='utf8')
        if process.returncode >= 0:
            splitted = process.stdout.strip().split(',')
            return int(splitted[0]), int(splitted[1]), float(splitted[2])
        print(f'Killed. {"trying again..." if i < RETRY_LIMIT - 1 else ""}')
    return -1, -1, -1


def solve_problems(conn, computed_problems, alg, weight, alternation_rate_forward=0, alternation_rate_backward=0):
    for pN in range(0, PROBLEMS_NUMBER):
        if (pN, alg, weight, alternation_rate_forward, alternation_rate_backward) in computed_problems:
            print(f'({pN=}, {alg=}, {weight=}, {alternation_rate_forward=}, {alternation_rate_backward=}) allready computed')
            continue
        print(f'computing ({pN=}, {alg=}, {weight=}, {alternation_rate_forward=}, {alternation_rate_backward=})...')
        res, nodes_expanded, time_elapsed = solve_problem(pN, alg, weight, alternation_rate_forward, alternation_rate_backward)
        save_result(conn, pN, alg, weight, res, nodes_expanded, time_elapsed, alternation_rate_forward, alternation_rate_backward)
        computed_problems.append((pN, alg, weight, alternation_rate_forward, alternation_rate_backward))


def problems_to_calculate():
    return len(WEIGHTS) * PROBLEMS_NUMBER * (1 + len(ALTERNATION_EQUAL) + 2 * len(ALTERNATION_DIFFERENT))


def test_alternation():
    conn = get_connection()
    create_tables(conn)
    computed_problems = conn.execute("""select problemNumber, algorithm, weight, alternationRateForward, alternationRateBackward from Result""").fetchall()
    print(f'allready benchmarked {len(computed_problems)} problems. remaining problems: {problems_to_calculate() - len(computed_problems)}')
    for w in WEIGHTS:
        print(f'************************** Weight: {w} *************************')
        print('benchmarking a*...')
        solve_problems(conn, computed_problems, ASTAR_ID, w)
        print('benchmarking mm with equal alternation rate...')
        for alternation_rate in ALTERNATION_EQUAL:
            solve_problems(conn, computed_problems, MM_ID, w, alternation_rate, alternation_rate)
        print('benchmarking mm with different alternation rates...')
        for alternation_rate_forward, alternation_rate_backward in ALTERNATION_DIFFERENT:
            solve_problems(conn, computed_problems, MM_ID, w, alternation_rate_forward, alternation_rate_backward)
            solve_problems(conn, computed_problems, MM_ID, w, alternation_rate_backward, alternation_rate_forward)


if __name__ == '__main__':
    test_alternation()
