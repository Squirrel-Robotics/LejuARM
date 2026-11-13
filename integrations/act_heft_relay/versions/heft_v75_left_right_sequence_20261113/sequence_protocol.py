"""One-way phase transition, no robot imports or publication side effects."""
from execution_core import finite, hands, hand_feedback, within

def validate_switch(packet, side, last_seq, previous, measured_home, measured_hand, reference, buffered_bytes=0):
    if (packet.get('op')!='switch_side' or side!='left' or packet.get('from_side')!='left'
            or packet.get('side')!='right'):
        raise ValueError('SEQUENCE_SWITCH_ORDER')
    if type(packet.get('after_seq')) is not int or packet['after_seq']!=last_seq or last_seq<1 or buffered_bytes:
        raise ValueError('SEQUENCE_SWITCH_NOT_DRAINED')
    home,hand=finite(packet.get('home'),21),hands(packet.get('hand_home'))
    if home!=previous['q'] or hand!=previous['hand']:
        raise ValueError('SEQUENCE_SWITCH_WOULD_JUMP')
    if not within(home,finite(measured_home,21),[0,13,14,15,16],1e-5):
        raise ValueError('SEQUENCE_LEFT_RETURN_COMMAND_INCOMPLETE')
    if max(abs(x) for x in hand[:6])>1e-7 or max(abs(x) for x in hand_feedback(measured_hand)[:6])>5.:
        raise ValueError('SEQUENCE_LEFT_RELEASE_UNCONFIRMED')
    # The inactive left reference must already be the published return endpoint.
    # Actual angular deviations retain the existing per-side reporting policy.
    if not within(finite(reference,21),home,range(21),.02):
        raise ValueError('SEQUENCE_NATIVE_BOUNDARY_NOT_REACHED')
    return home,hand
