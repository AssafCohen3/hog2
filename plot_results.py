import sqlite3
from collections import defaultdict
from pathlib import Path
import matplotlib.pyplot as plt
PLOTS_PATH = 'plots/'
DB_PATH = 'results.sqlite'
OPTIMAL_COSTS = [57, 55, 59, 56, 56, 52, 52, 50, 46, 59,
                 57, 45, 46, 59, 62, 42, 66, 55, 46, 52,
                 54, 59, 49, 54, 52, 58, 53, 52, 54, 47,
                 50, 59, 60, 52, 55, 52, 58, 53, 49, 54,
                 54, 42, 64, 50, 51, 49, 47, 49, 59, 53,
                 56, 56, 64, 56, 41, 55, 50, 51, 57, 66,
                 45, 57, 56, 51, 47, 61, 50, 51, 53, 52,
                 44, 56, 49, 56, 48, 57, 54, 53, 42, 57,
                 53, 62, 49, 55, 44, 45, 52, 65, 54, 50,
                 57, 57, 46, 53, 50, 49, 44, 54, 57, 54]
PROBLEMS_NUMBER = 100


def get_connection():
    return sqlite3.connect(DB_PATH)


def group_by(lst, key):
    to_ret = defaultdict(list)
    for i in lst:
        to_ret[key(i)].append(i)
    return to_ret


def plot_problem(plot_name, problem_number, variations_results):
    fig, ax = plt.subplots()
    for variation, variation_results in variations_results.items():
        variation_name = 'A*' if variation[0] == 1 else f'MM({variation[1]}, {variation[2]})'
        relative_costs = [OPTIMAL_COSTS[problem_number] / p[5] for p in variation_results]
        expanded_nodes = [p[6] for p in variation_results]
        ax.plot(expanded_nodes, relative_costs, marker='o', label=variation_name, alpha=0.7)
        # for r in variation_results:
        #     ax.annotate(r[2], (r[6], r[5] / OPTIMAL_COSTS[problem_number]))
    ax.set_ylabel('Relative quality')
    ax.set_xlabel('Expanded nodes')
    ax.set_xscale('log')
    ax.set_title(f'Problem {problem_number + 1}')
    ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    fig.tight_layout()
    path = f'{PLOTS_PATH}problem_{problem_number + 1}/'
    Path(path).mkdir(parents=True, exist_ok=True)
    fig.savefig(path + plot_name + '.png')
    plt.close(fig)


def plot():
    conn = get_connection()
    computed_problems = conn.execute("""
                    with WITH_MIN_RESULTS as (
                        select problemNumber, algorithm, weight, alternationRateForward, alternationRateBackward, result, expandedNodes, 
                            min(result) over (partition by problemNumber, algorithm, alternationRateForward, alternationRateBackward) as minimumVariationResult 
                        from Result where new=1
                    )
                    select problemNumber, algorithm, weight, alternationRateForward, alternationRateBackward, result, expandedNodes from WITH_MIN_RESULTS where minimumVariationResult > 0 order by problemNumber, algorithm, alternationRateForward, alternationRateForward, expandedNodes""").fetchall()
    problems_grouped = group_by(computed_problems, lambda p: p[0])
    for problem_number, problem_benchmarks in problems_grouped.items():
        variations_results = group_by(problem_benchmarks, lambda p: (p[1], p[3], p[4]))
        plot_problem('all_variations', problem_number, variations_results)


if __name__ == '__main__':
    plot()
