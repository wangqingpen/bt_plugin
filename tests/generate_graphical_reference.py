"""Build a golden command stream from the original graphical program, offline.

This generator never reads the BT XML to generate expectations and never sends
commands. Its expansion follows the deployed frontend's MoveBlend/queue rules.
"""
import hashlib
import json
from pathlib import Path
import sys


def chain(block):
    while block:
        yield block
        block = block.get('next', {}).get('block')


def lane(block):
    result = []
    for item in chain(block):
        kind, fields = item['type'], item.get('fields', {})
        if kind == 'not_run_execute_block':
            result.append('NotRunExecute')
        elif kind.startswith('moveabsj_block'):
            result.append(f"MoveAbsJ --jointtarget={fields['JOINTTARGET']} --speed=v5")
        elif kind.startswith('moveblend_block'):
            result.append('MoveBlend --type=first_insert')
            result.extend(lane(item['inputs']['DO']['block']))
            result.append('MoveBlend --type=start')
        elif kind.startswith('moveblendl_block'):
            result.append(f"MoveBlend --type=insert_line --robottarget={fields['ROBOTTARGET']}"
                          f" --ratio={fields.get('RATIO') or '5'} --speed=v5"
                          f" --zone={fields.get('ZONE') or '{0,0}'} --is_optimal={fields.get('IS_OPTIMAL', '0')}")
        elif kind.startswith('raw_command_block'):
            args = []
            for key, name in fields.items():
                if not key.startswith('PARAMETER_NAME'):
                    continue
                raw = fields['PARAMETER_VALUE' + key[len('PARAMETER_NAME'):]].strip()
                try:
                    value = json.loads(raw)
                except ValueError:
                    value = raw
                if isinstance(value, (dict, list, bool)) or value is None:
                    value = json.dumps(value, ensure_ascii=False, separators=(',', ':'))
                args.append(f'--{name.strip()}={value}')
            result.append(' '.join([fields['EVENT_NAME'].strip(), *args]))
        else:
            raise ValueError(f'Unsupported graphical block: {kind}')
    return result


def generate(source):
    program = json.loads(source.read_text(encoding='utf-8'))['blocks']['blocks'][0]
    queues = [[], [], []]
    rows = 0
    for row in chain(program.get('next', {}).get('block')):
        assert row['type'] == 'robot_wrap_command_block'
        parts = [lane(row['inputs'].get(f'ROBOT_WRAP_COMMAND{i}', {}).get('block')) or ['NotRunExecute'] for i in range(3)]
        assert len(set(map(len, parts))) == 1, f'Unaligned graphical row {rows+1}'
        for queue, commands in zip(queues, parts):
            queue.extend(commands)
        rows += 1
    assert len(set(map(len, queues))) == 1
    frames = [{'command': '{' + '||'.join(slots) + '}'} for slots in zip(*queues)]
    return frames, {'program': program['fields']['FIELD_PROGRAM_NAME'], 'rows': rows,
                    'frames': len(frames), 'speed_override': 'v5',
                    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(), 'robot_commands_sent': 0}


if __name__ == '__main__':
    source, target = map(Path, sys.argv[1:3])
    frames, evidence = generate(source)
    target.write_text(json.dumps(frames, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    target.with_suffix('.provenance.json').write_text(json.dumps(evidence, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(evidence))
