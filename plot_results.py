import sqlite3
from collections import defaultdict
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

PLOTS_PATH = 'plots/'
DB_PATH = 'resultssave.sqlite'
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
MARKERS = {
    1: '8',
    1.1: 's',
    2: 'p',
    2.5: '*',
    5: 'D',
    10: '^',
    20: 'P'
}

def get_connection():
    return sqlite3.connect(DB_PATH)


def create_optimal_costs(conn):
    conn.execute("""
    create table if not exists OptimalCosts(
        problemNumber integer not null primary key,
        optimalCost integer not null
    )
    """)
    conn.executemany("""insert or ignore into OptimalCosts (problemNumber, optimalCost) values (?, ?)""", [(i, v) for i, v in enumerate(OPTIMAL_COSTS)])
    conn.commit()

def group_by(lst, key):
    to_ret = defaultdict(list)
    for i in lst:
        to_ret[key(i)].append(i)
    return to_ret


def get_table_vals(df):
    quality_cols = pd.get_dummies(df.weight).mul(df['resultQuality'], 0)
    expanded_nodes_cols = pd.get_dummies(df.weight).mul(df['expandedNodes'], 0)
    quality_df = pd.concat([df[['algorithm', 'alternationRateForward', 'alternationRateBackward']], quality_cols], axis=1).groupby(
        ['algorithm', 'alternationRateForward', 'alternationRateBackward']).sum().reset_index()
    nodes_df = pd.concat([df[['algorithm', 'alternationRateForward', 'alternationRateBackward']], expanded_nodes_cols], axis=1).groupby(
        ['algorithm', 'alternationRateForward', 'alternationRateBackward']).sum().reset_index()

    quality_df['alg'] = quality_df[['algorithm', 'alternationRateForward', 'alternationRateBackward']].\
        apply(lambda x: 'A* - Q' if x[0] == 1 else f'MM({x[1]}, {x[2]}) - Q', axis=1)
    nodes_df['alg'] = nodes_df[['algorithm', 'alternationRateForward', 'alternationRateBackward']].\
        apply(lambda x: 'A* - Ns' if x[0] == 1 else f'MM({x[1]}, {x[2]}) - Ns', axis=1)
    new_df = pd.concat([nodes_df, quality_df], axis=0)
    new_df = new_df.drop(['algorithm', 'alternationRateForward', 'alternationRateBackward'], axis=1)
    cols = new_df.columns.tolist()
    cols = ['alg'] + cols[:-1]
    return new_df[cols]

def plot_algs_problem(conn, pn, algorithms):
    pd.set_option("display.max_rows", None, "display.max_columns", None)
    sql = f"""
                    with WITH_MIN_RESULTS as (
                        select problemNumber, algorithm, weight, alternationRateForward, alternationRateBackward, result, expandedNodes, 
                            min(result) over (partition by problemNumber, algorithm, alternationRateForward, alternationRateBackward) as minimumVariationResult 
                        from Result where new=1 and problemNumber = ? and  (algorithm, alternationRateForward, alternationRateBackward) in (VALUES {','.join(f'({",".join("?" * len(t))})' for t in algorithms)}
                        ))
                    select algorithm, 
                           weight, alternationRateForward, alternationRateBackward, result, OC.optimalCost / cast(WITH_MIN_RESULTS.result as real) as resultQuality, expandedNodes 
                    from WITH_MIN_RESULTS 
                    inner join OptimalCosts OC on OC.problemNumber = WITH_MIN_RESULTS.problemNumber
                    where minimumVariationResult > 0 
                    order by WITH_MIN_RESULTS.problemNumber, algorithm, alternationRateForward, alternationRateForward, expandedNodes"""
    params = [pn] + [spec for alg in algorithms for spec in alg]
    df = pd.read_sql_query(sql, conn, params=params)
    if len(df.index) == 0:
        return
    fig, ax = plt.subplots()
    colors = {}
    for (alg, alt_forward, alt_backward), grp in df.groupby(['algorithm', 'alternationRateForward', 'alternationRateBackward']):
        ax = grp.plot(ax=ax, kind='line', x='expandedNodes', y='resultQuality', label='WA*' if alg == 1 else f'MM({alt_forward}, {alt_backward})', alpha=0.7)
        colors[(alg, alt_forward, alt_backward)] = plt.gca().lines[-1].get_color()
    for (alg, alt_forward, alt_backward, weight), grp in df.groupby(['algorithm', 'alternationRateForward', 'alternationRateBackward', 'weight']):
        ax = grp.plot(ax=ax, kind='scatter', x='expandedNodes', y='resultQuality', marker=MARKERS[weight], c=colors[(alg, alt_forward, alt_backward)])
    ax.autoscale_view()
    legend_elements = [Line2D([0], [0], marker=m, label=w, linestyle='None') for w, m in MARKERS.items()]
    legend1 = ax.legend(loc='lower left', bbox_to_anchor=(1, 0.5), title="Algorithms")
    legend2 = plt.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1, 0.5), title="Weights markers")
    ax.add_artist(legend2)
    ax.add_artist(legend1)
    ttt = get_table_vals(df)
    plt.table(cellText=ttt.values, colLabels=ttt.columns, loc='bottom')
    ax.set_xscale('log')
    plt.tick_params(axis='x', which='both', bottom=False, top=True, labelbottom=False, labeltop=True)
    ax.xaxis.set_label_position('top')
    _ = plt.grid()
    _ = plt.ylim(0, df['resultQuality'].max() * 1.25)
    _ = plt.ylabel('resultQuality')
    plt.title('Problem ' + str(pn + 1))
    plt.tight_layout()
    plt.subplots_adjust(right=0.75)
    path = f'{PLOTS_PATH}problem_{pn + 1}/'
    Path(path).mkdir(parents=True, exist_ok=True)
    names = '_'.join(str(a) for alg in algorithms for a in alg)
    plt.savefig(path + f'algs_' + names + '.png', dpi=300)
    plt.close()

def plot_algs_avg(conn, algorithms):
    pd.set_option("display.max_rows", None, "display.max_columns", None)
    sql = f"""
                    with WITH_MIN_RESULTS as (
                        select problemNumber, algorithm, weight, alternationRateForward, alternationRateBackward, result, expandedNodes, 
                            min(result) over (partition by problemNumber, algorithm, alternationRateForward, alternationRateBackward) as minimumVariationResult 
                        from Result where new=1 and (algorithm, alternationRateForward, alternationRateBackward) in (VALUES {','.join(f'({",".join("?" * len(t))})' for t in algorithms)}
                        ))
                    select algorithm, 
                           weight, alternationRateForward, alternationRateBackward, avg(expandedNodes) as expandedNodes, avg(OC.optimalCost / cast(WITH_MIN_RESULTS.result as real)) as resultQuality
                    from WITH_MIN_RESULTS 
                    inner join OptimalCosts OC on OC.problemNumber = WITH_MIN_RESULTS.problemNumber
                    where minimumVariationResult > 0 
                    group by algorithm, weight, alternationRateForward, alternationRateBackward
                    order by algorithm, alternationRateForward, alternationRateForward, expandedNodes"""
    params = [spec for alg in algorithms for spec in alg]
    df = pd.read_sql_query(sql, conn, params=params)
    fig, ax = plt.subplots()
    colors = {}
    for (alg, alt_forward, alt_backward), grp in df.groupby(['algorithm', 'alternationRateForward', 'alternationRateBackward']):
        ax = grp.plot(ax=ax, kind='line', x='expandedNodes', y='resultQuality', label='WA*' if alg == 1 else f'MM({alt_forward}, {alt_backward})', alpha=0.7)
        colors[(alg, alt_forward, alt_backward)] = plt.gca().lines[-1].get_color()
    for (alg, alt_forward, alt_backward, weight), grp in df.groupby(['algorithm', 'alternationRateForward', 'alternationRateBackward', 'weight']):
        ax = grp.plot(ax=ax, kind='scatter', x='expandedNodes', y='resultQuality', marker=MARKERS[weight], c=colors[(alg, alt_forward, alt_backward)])
    ax.autoscale_view()
    legend_elements = [Line2D([0], [0], marker=m, label=w, linestyle='None') for w, m in MARKERS.items()]
    legend1 = ax.legend(loc='lower left', bbox_to_anchor=(1, 0.5), title="Algorithms")
    legend2 = plt.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1, 0.5), title="Weights markers")
    ax.add_artist(legend2)
    ax.add_artist(legend1)
    ttt = get_table_vals(df)
    plt.table(cellText=ttt.values, colLabels=ttt.columns, loc='bottom')
    ax.set_xscale('log')
    plt.tick_params(axis='x', which='both', bottom=False, top=True, labelbottom=False, labeltop=True)
    ax.xaxis.set_label_position('top')
    _ = plt.grid()
    _ = plt.ylim(0, df['resultQuality'].max() * 1.25)
    _ = plt.ylabel('resultQuality')
    plt.title('Averages')
    plt.tight_layout()
    plt.subplots_adjust(right=0.75)
    path = f'{PLOTS_PATH}averages/'
    Path(path).mkdir(parents=True, exist_ok=True)
    names = '_'.join(str(a) for alg in algorithms for a in alg)
    plt.savefig(path + f'algs_' + names + '.png', dpi=300)
    plt.close()


def plot_all():
    conn = get_connection()
    for i in range(0, 99):
        plot_algs_problem(conn, i, [(1, 0, 0), (2, 1, 1), (2, 2, 1)])
    plot_algs_avg(conn, [(1, 0, 0), (2, 1, 1), (2, 2, 1)])


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
    plot_algs_avg(get_connection(), [(1, 0, 0)])
    #plot_all()
    #create_optimal_costs(conn)
